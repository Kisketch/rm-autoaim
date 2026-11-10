// Camera model and armor pose estimation.
//
// Pipeline per armor:
//   1. IPPE PnP (two solutions, rm_auto_aim / FYT2024), pick the lower
//      reprojection error, break near-ties with the expected +15 deg armor pitch.
//   2. Optional yaw refinement (idea of FYT2024's BA solver and SJTU/Tongji
//      "fixed-pitch" tricks): with the armor pitch known (15 deg) and roll 0,
//      search the single remaining angle - yaw - that minimises the corner
//      reprojection error. PnP yaw on a 50 px plate is very noisy; this is not.
#pragma once

#include <opencv2/core.hpp>

#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

class CameraModel {
public:
    explicit CameraModel(const CameraConfig& cfg);
    const cv::Matx33d& K() const { return K_; }
    const cv::Mat& dist() const { return dist_; }
    const cv::Matx33d& R_world_cam() const { return R_wc_; }
    cv::Point2d project(const cv::Vec3d& world) const;  // world point -> pixel
    bool inFront(const cv::Vec3d& world) const;
    double fx() const { return K_(0, 0); }

private:
    cv::Matx33d K_;
    cv::Mat dist_;
    cv::Matx33d R_wc_;
    // Cached intrinsics + distortion for the allocation-free projection fast
    // path (distortion models with <= 5 coefficients; longer models fall back
    // to cv::projectPoints). Numerically identical to projectPoints for the
    // 5-coefficient model (verified to <5e-12 px on 10k random points).
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    double dc_[5] = {0, 0, 0, 0, 0};
    bool fast_projection_ = false;
};

// R_world_armor for a given yaw and armor pitch (roll 0).
cv::Matx33d armorRotation(double yaw, double pitch);
// The 4 corner points (light-bar end points) of an armor in the world frame,
// same order as ArmorDetection::corners.
std::array<cv::Vec3d, 4> armorCorners(const cv::Vec3d& center, double yaw, double pitch,
                                      ArmorType type);

class PoseSolver {
public:
    PoseSolver(const CameraModel& camera, const PoseConfig& cfg);
    bool solve(const ArmorDetection& det, ArmorObservation& out) const;
    std::vector<ArmorObservation> solveAll(const std::vector<ArmorDetection>& dets) const;
    const CameraModel& camera() const { return camera_; }

private:
    double reprojectionError(const ArmorDetection& det, const cv::Vec3d& center, double yaw,
                             double pitch) const;
    const CameraModel& camera_;
    PoseConfig cfg_;
};

}  // namespace autoaim
