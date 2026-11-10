#include "autoaim/draw.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>

namespace autoaim {
namespace {

void text(cv::Mat& img, const std::string& s, cv::Point p, cv::Scalar c, double scale = 0.6) {
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, c, 1, cv::LINE_AA);
}

cv::Point toPx(const cv::Point2d& p) { return {cvRound(p.x), cvRound(p.y)}; }

}  // namespace

void drawOverlay(cv::Mat& img, const FrameObservations& obs, const FrameLog& log, const RobotTracker& geom,
                 const CameraModel& camera) {
    // detections (green) with panel outline
    for (const auto& a : obs.armors) {
        std::vector<cv::Point> poly;
        for (const auto& p : a.det.corners) poly.push_back(toPx(p));
        cv::polylines(img, poly, true, {0, 255, 0}, 1, cv::LINE_AA);
        std::vector<cv::Point> panel;
        for (const auto& p : panelPolygon(a.det)) panel.push_back(toPx(p));
        cv::polylines(img, panel, true, {0, 160, 0}, 1, cv::LINE_AA);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%s %.1fm %+.0fdeg", a.det.number.c_str(), a.distance,
                      a.yaw * 180 / kPi);
        text(img, buf, toPx(a.det.corners[1]) + cv::Point(0, -6), {0, 255, 0}, 0.45);
    }

    const bool active = log.state == TrackState::TRACKING || log.state == TrackState::TEMP_LOST;
    const auto& x = log.x;
    if (active) {
        // robot model at the current time: all armors (cyan = facing camera)
        const double center_dir = std::atan2(x[RobotTracker::YC], x[RobotTracker::XC]);
        for (int k = 0; k < geom.armorsNum(); ++k) {
            const double yaw = geom.armorYaw(x, k);
            const bool facing = std::abs(wrapAngle(yaw - center_dir)) < kPi / 2;
            const auto corners = armorCorners(geom.armorPosition(x, k), yaw, 15 * kPi / 180, geom.armor_type);
            std::vector<cv::Point> poly;
            bool ok = true;
            for (const auto& c : corners) {
                if (!camera.inFront(c)) ok = false;
                poly.push_back(toPx(camera.project(c)));
            }
            if (!ok) continue;
            cv::polylines(img, poly, true, facing ? cv::Scalar(255, 255, 0) : cv::Scalar(120, 120, 120), 1,
                          cv::LINE_AA);
        }
        const cv::Vec3d c(x[RobotTracker::XC], x[RobotTracker::YC], x[RobotTracker::ZC]);
        if (camera.inFront(c)) cv::drawMarker(img, toPx(camera.project(c)), {0, 255, 255}, cv::MARKER_CROSS, 14, 2);
    }
    if (log.cmd.valid && camera.inFront(log.cmd.aim_point)) {
        cv::circle(img, toPx(camera.project(log.cmd.aim_point)), 7,
                   log.cmd.center_mode ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    if (log.shot && log.shot_rec.result != ShotResult::UNKNOWN) {
        const bool hit = log.shot_rec.result == ShotResult::HIT;
        cv::drawMarker(img, toPx(log.shot_rec.impact_px), hit ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
                       cv::MARKER_TILTED_CROSS, 10, 2);
    }

    // status text
    char buf[160];
    std::snprintf(buf, sizeof(buf), "frame %d  %s  matched %d", log.frame, toString(log.state), log.matched);
    text(img, buf, {10, 25}, {255, 255, 255});
    if (active) {
        std::snprintf(buf, sizeof(buf), "w %+.2f rad/s  r1 %.2f  r2 %.2f  dz %+.3f  d %.2fm", x[RobotTracker::W],
                      x[RobotTracker::R1], x[RobotTracker::R2], x[RobotTracker::DZ],
                      std::hypot(x[RobotTracker::XC], x[RobotTracker::YC]));
        text(img, buf, {10, 50}, {255, 255, 0});
    }
    if (log.cmd.valid) {
        std::snprintf(buf, sizeof(buf), "%s  yaw %+.2f pitch %+.2f deg  t_fly %.0f ms  facing %+.0f  %s",
                      log.cmd.center_mode ? "CENTER" : "ARMOR", log.cmd.yaw * 180 / kPi, log.cmd.pitch * 180 / kPi,
                      log.cmd.t_fly * 1000, log.cmd.facing_deg,
                      log.shot ? (log.shot_rec.result == ShotResult::HIT
                                      ? "FIRE: HIT"
                                      : (log.shot_rec.result == ShotResult::MISS ? "FIRE: MISS" : "FIRE: ?"))
                               : "hold");
        text(img, buf, {10, 75}, log.shot ? cv::Scalar(0, 200, 255) : cv::Scalar(200, 200, 200));
    }

    // top-down inset (x forward -> up in the inset, y left -> left)
    if (active) {
        const int S = 220;
        const cv::Rect roi(img.cols - S - 10, 10, S, S);
        cv::Mat sub = img(roi);
        sub *= 0.35;
        cv::rectangle(img, roi, {255, 255, 255}, 1);
        const cv::Point2d c(x[RobotTracker::XC], x[RobotTracker::YC]);
        const double scale = S / 1.2;  // 1.2 m across
        auto map = [&](double wx, double wy) {
            return cv::Point(roi.x + S / 2 + cvRound(-(wy - c.y) * scale), roi.y + S / 2 + cvRound(-(wx - c.x) * scale));
        };
        // line of sight from the camera
        const double los = std::atan2(c.y, c.x);
        cv::line(img, map(c.x - 0.6 * std::cos(los), c.y - 0.6 * std::sin(los)), map(c.x, c.y), {90, 90, 90}, 1);
        cv::drawMarker(img, map(c.x, c.y), {0, 255, 255}, cv::MARKER_CROSS, 10, 1);
        for (int k = 0; k < geom.armorsNum(); ++k) {
            const cv::Vec3d p = geom.armorPosition(x, k);
            const double yaw = geom.armorYaw(x, k);
            const double hw = 0.07;
            const cv::Point a = map(p[0] - hw * std::sin(yaw), p[1] + hw * std::cos(yaw));
            const cv::Point b = map(p[0] + hw * std::sin(yaw), p[1] - hw * std::cos(yaw));
            const bool chosen = log.cmd.valid && k == log.cmd.armor_k;
            cv::line(img, a, b, chosen ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 255, 0), chosen ? 3 : 2);
        }
        for (const auto& a : obs.armors) {
            if (deviceId(a.det) != geom.id() && !geom.id().empty()) continue;
            cv::circle(img, map(a.position[0], a.position[1]), 3, {0, 255, 0}, -1);
        }
        text(img, "top view (1.2 m)", {roi.x + 5, roi.y + S - 8}, {220, 220, 220}, 0.4);
    }
}

}  // namespace autoaim
