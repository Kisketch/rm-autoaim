// Target tracking.
//
// RobotTracker - whole-robot EKF ("整车观测"). Based on the 9-state model of
// chenjunnn/rm_auto_aim (MIT) and the d_z extension of FYT2024 (Apache-2.0),
// re-implemented with these changes:
//   * 11 states: xc vxc yc vyc zc vzc yaw w r1 r2 dz. Both radii and the height
//     offset of the second armor pair live inside the filter, so they are
//     estimated instead of swapped by hand on every "armor jump".
//   * Multi-armor association: each observation is matched to the nearest of
//     the N predicted armors (yaw + 2*pi*k/N); the measurement model is that
//     armor's. Two plates seen in the same frame are fused sequentially, which
//     pins down the centre and radius quickly. No special jump handling needed.
//   * Anisotropic measurement noise: lateral sigma = d * px / f, depth sigma =
//     d * px / bar_length_px (depth comes from apparent size and is ~10x worse).
//   * Mahalanobis gating; yaw residuals are wrapped (rm_auto_aim did not).
//
// PointTracker - baseline: constant-velocity KF on the plate centre only
// (what many simple predictors do). Used to quantify the gain of the robot model.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "autoaim/config.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

enum class TrackState { LOST, DETECTING, TRACKING, TEMP_LOST };
const char* toString(TrackState s);

class RobotTracker {
public:
    static constexpr int N = 13;
    enum Idx { XC = 0, VXC, YC, VYC, ZC, VZC, YAW, W, R1, R2, DZ, AXC, AY };
    using Vec = cv::Vec<double, N>;
    using Mat = cv::Matx<double, N, N>;

    RobotTracker(const TrackerConfig& cfg, const CameraModel& camera);

    void update(const FrameObservations& frame);
    void reset() { state_ = TrackState::LOST; }

    TrackState state() const { return state_; }
    bool active() const { return state_ == TrackState::TRACKING || state_ == TrackState::TEMP_LOST; }
    const Vec& x() const { return x_; }
    const Mat& P() const { return P_; }
    double stamp() const { return stamp_; }
    const std::string& id() const { return id_; }
    int armorsNum() const { return cfg_.armors_num; }

    // State extrapolated to time t (constant velocity / constant angular velocity).
    Vec predict(double t) const;
    // Covariance extrapolated to time t. Without process noise it measures how
    // well the *current* state is known (Q is tuned for responsiveness, so the
    // full prediction covariance is not a calibrated uncertainty).
    Mat predictCovariance(double t, bool with_process_noise = true) const;
    // d armorPosition / d x
    cv::Matx<double, 3, N> armorJacobian(const Vec& x, int k) const;
    double qScale() const { return q_scale_; }
    // Armor k of a state: position, yaw.
    cv::Vec3d armorPosition(const Vec& x, int k) const;
    double armorYaw(const Vec& x, int k) const;
    double armorRadius(const Vec& x, int k) const;

    // diagnostics of the last update
    int matched_count = 0;
    double last_nis = 0;
    ArmorType armor_type = ArmorType::SMALL;

private:
    void init(const ArmorObservation& obs, double stamp);
    void transition(double dt, Mat& F, Mat& Q) const;
    void predictTo(double t);
    bool updateWith(const ArmorObservation& obs, int k, double& nis_pos);
    cv::Matx<double, 4, 4> measurementNoise(const ArmorObservation& obs) const;
    void clampState();

    TrackerConfig cfg_;
    const CameraModel& camera_;
    TrackState state_ = TrackState::LOST;
    Vec x_;
    Mat P_;
    double stamp_ = 0;
    double last_match_stamp_ = 0;
    std::string id_;
    int detect_count_ = 0;
    int unmatched_with_target_ = 0;
    double nis_ema_ = 3.0;
    double q_scale_ = 1.0;
    // Gate widening when frames are far apart (dropped frames, lower fps):
    // gates are tuned for 40 ms, a 120 ms gap needs ~3x the radius.
    double match_dt_scale_ = 1.0;
};

class PointTracker {
public:
    using Vec = cv::Vec<double, 6>;  // x vx y vy z vz
    using Mat = cv::Matx<double, 6, 6>;
    PointTracker(const TrackerConfig& cfg, const CameraModel& camera);
    void update(const FrameObservations& frame);
    bool active() const { return active_; }
    Vec predict(double t) const;
    cv::Vec3d position(const Vec& x) const { return {x[0], x[2], x[4]}; }
    double stamp() const { return stamp_; }
    const std::string& id() const { return id_; }
    double yaw() const { return yaw_; }
    ArmorType armor_type = ArmorType::SMALL;

private:
    void init(const ArmorObservation& obs, double stamp);
    TrackerConfig cfg_;
    const CameraModel& camera_;
    bool active_ = false;
    Vec x_;
    Mat P_;
    double stamp_ = 0, last_match_ = 0, yaw_ = 0;
    std::string id_;
};

// Picks the observation the tracker should lock on: same id if possible, else
// the one closest to the image centre.
int chooseInitialTarget(const std::vector<ArmorObservation>& armors);

}  // namespace autoaim
