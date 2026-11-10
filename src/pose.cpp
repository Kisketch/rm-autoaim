#include "autoaim/pose.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <cmath>

namespace autoaim {

namespace {
// camera optical (X right, Y down, Z forward) -> world (x forward, y left, z up)
const cv::Matx33d kWorldFromOptical(0, 0, 1, -1, 0, 0, 0, -1, 0);

cv::Matx33d rotZ(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, -s, 0, s, c, 0, 0, 0, 1};
}
cv::Matx33d rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return {c, 0, s, 0, 1, 0, -s, 0, c};
}
}  // namespace

CameraModel::CameraModel(const CameraConfig& cfg) {
    K_ = cfg.K();
    dist_ = cv::Mat(cfg.dist, true).reshape(1, 1);
    // positive pitch looks down: rotate the forward axis about world +y
    R_wc_ = rotY(cfg.pitch_deg * kPi / 180.0) * kWorldFromOptical;
    fx_ = K_(0, 0), fy_ = K_(1, 1), cx_ = K_(0, 2), cy_ = K_(1, 2);
    fast_projection_ = dist_.total() <= 5;
    if (fast_projection_) {
        for (int i = 0; i < (int)dist_.total(); ++i) dc_[i] = dist_.at<double>(0, i);
    }
}

bool CameraModel::inFront(const cv::Vec3d& world) const {
    const cv::Vec3d c = R_wc_.t() * world;
    return c[2] > 0.05;
}

cv::Point2d CameraModel::project(const cv::Vec3d& world) const {
    const cv::Vec3d c = R_wc_.t() * world;
    if (fast_projection_) {
        // Direct pinhole + (k1,k2,p1,p2,k3) distortion, the same math
        // cv::projectPoints performs for the 5-coefficient model, without
        // its per-call vector/Mat overhead. This runs ~800x per armor solve
        // (yaw scan + refinement), so the overhead dominated pose time.
        const double X = c[0], Y = c[1], Z = c[2];
        if (std::abs(Z) < 1e-12) return {-1e9, -1e9};
        const double x = X / Z, y = Y / Z;
        const double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
        const double radial = 1.0 + dc_[0] * r2 + dc_[1] * r4 + dc_[4] * r6;
        const double xp = x * radial + 2.0 * dc_[2] * x * y + dc_[3] * (r2 + 2.0 * x * x);
        const double yp = y * radial + dc_[2] * (r2 + 2.0 * y * y) + 2.0 * dc_[3] * x * y;
        return {fx_ * xp + cx_, fy_ * yp + cy_};
    }
    std::vector<cv::Point3d> obj{{c[0], c[1], c[2]}};
    std::vector<cv::Point2d> img;
    cv::projectPoints(obj, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K_, dist_, img);
    return img[0];
}

cv::Matx33d armorRotation(double yaw, double pitch) { return rotZ(yaw) * rotY(pitch); }

std::array<cv::Vec3d, 4> armorCorners(const cv::Vec3d& center, double yaw, double pitch,
                                      ArmorType type) {
    const ArmorGeometry& g = geometryOf(type);
    const double hw = g.light_width / 2, hh = g.light_height / 2;
    // armor frame: x into the robot, y to the left (seen from the camera), z up
    const cv::Vec3d local[4] = {{0, hw, -hh}, {0, hw, hh}, {0, -hw, hh}, {0, -hw, -hh}};
    const cv::Matx33d R = armorRotation(yaw, pitch);
    std::array<cv::Vec3d, 4> out;
    for (int i = 0; i < 4; ++i) out[i] = center + R * local[i];
    return out;
}

PoseSolver::PoseSolver(const CameraModel& camera, const PoseConfig& cfg)
    : camera_(camera), cfg_(cfg) {}

double PoseSolver::reprojectionError(const ArmorDetection& det, const cv::Vec3d& center, double yaw,
                                     double pitch) const {
    const auto corners = armorCorners(center, yaw, pitch, det.type);
    double err = 0;
    for (int i = 0; i < 4; ++i) {
        const cv::Point2d p = camera_.project(corners[i]);
        const cv::Point2d d = p - cv::Point2d(det.corners[i]);
        err += std::sqrt(d.dot(d));
    }
    return err / 4.0;
}

bool PoseSolver::solve(const ArmorDetection& det, ArmorObservation& out) const {
    const ArmorGeometry& g = geometryOf(det.type);
    const double hw = g.light_width / 2, hh = g.light_height / 2;
    const std::vector<cv::Point3d> obj{{0, hw, -hh}, {0, hw, hh}, {0, -hw, hh}, {0, -hw, -hh}};
    const std::vector<cv::Point2d> img{cv::Point2d(det.corners[0]), cv::Point2d(det.corners[1]),
                                       cv::Point2d(det.corners[2]), cv::Point2d(det.corners[3])};
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> errs;
    cv::Mat rv, tv;
    int n = 0;
    try {
        n = cv::solvePnPGeneric(obj, img, camera_.K(), camera_.dist(), rvecs, tvecs, false,
                                cv::SOLVEPNP_IPPE, rv, tv, errs);
    } catch (const cv::Exception&) {
        return false;
    }
    if (n <= 0) return false;

    const double expected_pitch = cfg_.armor_pitch_deg * kPi / 180.0;
    int best = 0;
    double best_cost = 1e18;
    double best_yaw = 0;
    cv::Vec3d best_t;
    for (int i = 0; i < n; ++i) {
        cv::Matx33d R_ca;
        cv::Rodrigues(rvecs[i], R_ca);
        const cv::Matx33d R_wa = camera_.R_world_cam() * R_ca;
        const double yaw = std::atan2(R_wa(1, 0), R_wa(0, 0));
        const double pitch = std::asin(std::clamp(-R_wa(2, 0), -1.0, 1.0));
        const cv::Vec3d t_c(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
        const double err = errs.size() > static_cast<std::size_t>(i) ? errs[i] : 0.0;
        // reprojection error dominates; pitch prior breaks the IPPE near-tie
        const double cost = err + 2.0 * std::abs(pitch - expected_pitch);
        if (cost < best_cost) {
            best_cost = cost, best = i, best_yaw = yaw;
            best_t = camera_.R_world_cam() * t_c;
        }
    }
    (void)best;

    out.det = det;
    out.position = best_t;
    out.distance = cv::norm(best_t);
    out.yaw = best_yaw;

    if (cfg_.optimize_yaw) {
        // Fixed pitch, search yaw around the line of sight. Coarse-to-fine:
        // a 2 deg grid (the reprojection-error curve in yaw is smooth on
        // that scale for a 4-point planar target), then golden-section
        // refinement over the winning +-2 deg bracket. Halves the scan
        // evaluations vs the previous 1 deg grid + +-1 deg refinement;
        // final yaw precision is unchanged (bracket shrinks to ~0.002 deg).
        const double los = std::atan2(best_t[1], best_t[0]);
        const double range = cfg_.yaw_search_range_deg * kPi / 180.0;
        const double step = 2.0 * kPi / 180.0;
        double y_best = best_yaw, e_best = 1e18;
        for (double dy = -range; dy <= range + 1e-9; dy += step) {
            const double e = reprojectionError(det, best_t, los + dy, expected_pitch);
            if (e < e_best) e_best = e, y_best = los + dy;
        }
        // golden-section refinement within +-2 deg
        double a = y_best - step, b = y_best + step;
        const double gr = 0.6180339887498949;
        double c = b - gr * (b - a), d = a + gr * (b - a);
        double fc = reprojectionError(det, best_t, c, expected_pitch);
        double fd = reprojectionError(det, best_t, d, expected_pitch);
        // 16 iterations shrink the 2 deg bracket to ~0.001 deg, far below
        // the corner noise; 20 bought nothing measurable.
        for (int it = 0; it < 16; ++it) {
            if (fc < fd) {
                b = d, d = c, fd = fc;
                c = b - gr * (b - a);
                fc = reprojectionError(det, best_t, c, expected_pitch);
            } else {
                a = c, c = d, fc = fd;
                d = a + gr * (b - a);
                fd = reprojectionError(det, best_t, d, expected_pitch);
            }
        }
        out.yaw = wrapAngle(0.5 * (a + b));
        out.reproj_error = reprojectionError(det, best_t, out.yaw, expected_pitch);
    } else {
        out.reproj_error = reprojectionError(det, best_t, out.yaw, expected_pitch);
    }

    if (cfg_.yaw_gain != 1.0) {
        const double los = std::atan2(best_t[1], best_t[0]);
        out.yaw = wrapAngle(los + cfg_.yaw_gain * wrapAngle(out.yaw - los));
    }

    const cv::Point2d c(camera_.K()(0, 2), camera_.K()(1, 2));
    out.image_center_dist = cv::norm(cv::Point2d(det.center) - c);
    return std::isfinite(out.position[0]) && out.distance > 0.2 && out.distance < 30.0;
}

std::vector<ArmorObservation> PoseSolver::solveAll(const std::vector<ArmorDetection>& dets) const {
    // Each armor solves independently (solve() is const and stateless), so
    // run them in parallel; results keep input order and failed solves are
    // dropped exactly as in the sequential version.
    std::vector<ArmorObservation> solved(dets.size());
    std::vector<char> ok(dets.size(), 0);
    cv::parallel_for_(cv::Range(0, static_cast<int>(dets.size())), [&](const cv::Range& range) {
        for (int i = range.start; i < range.end; ++i) {
            if (solve(dets[i], solved[i])) ok[i] = 1;
        }
    });
    std::vector<ArmorObservation> out;
    out.reserve(dets.size());
    for (std::size_t i = 0; i < dets.size(); ++i)
        if (ok[i]) out.push_back(solved[i]);
    return out;
}

}  // namespace autoaim
