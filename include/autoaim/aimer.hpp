// Aim-point selection, gimbal command and fire decision.
//
// RobotAimer (whole-robot model):
//   - fixed-point iteration over latency + time of flight (FYT2024 / rm_vision);
//   - picks the armor whose facing angle is closest to the camera, shifted
//     towards the incoming plate by side_angle when spinning (FYT2024);
//   - above center_mode_omega it aims at the robot centre and only fires when
//     some plate will face the barrel at impact time (classic anti-spin
//     strategy: the gimbal stops chasing plates and waits for them);
//   - fires only when the simulated gimbal is within the plate's angular size.
// PointAimer / DirectAimer are the baselines (point KF / no prediction).
#pragma once

#include <opencv2/core.hpp>

#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/tracker.hpp"
#include "autoaim/types.hpp"

namespace autoaim {

struct AimCommand {
    bool valid = false;
    bool fire = false;
    bool center_mode = false;
    double yaw = 0, pitch = 0;  // desired gimbal angles (rad)
    // Rate of the commanded angles (rad/s), for gimbals that accept a
    // velocity feedforward term next to the angle (B-01).  Filled by the
    // aimers from successive commands; 0 when no previous command exists.
    double yaw_rate = 0, pitch_rate = 0;
    cv::Vec3d aim_point{0, 0, 0};
    double t_fly = 0;
    double t_impact = 0;        // absolute time
    int armor_k = -1;
    double facing_deg = 0;
    double distance = 0;
    double sigma_lat = 0;       // predicted lateral std of the aimed plate at impact (m)
};

// First-order gimbal with a rate limit - enough to make "chasing every plate"
// visibly worse than "waiting at the centre".
class GimbalSim {
public:
    GimbalSim(double tau, double max_rate, double ff_gain = 0.0)
        : tau_(tau), max_rate_(max_rate), ff_(ff_gain) {}
    void step(double cmd_yaw, double cmd_pitch, double dt);
    double yaw = 0, pitch = 0;
    bool initialised = false;

private:
    double tau_, max_rate_, ff_ = 0.0;
    double prev_yaw_ = 0, prev_pitch_ = 0;
    bool has_prev_ = false;
};

bool onTarget(const GimbalSim& g, const AimCommand& cmd, ArmorType type, double scale);

// Rate of successive aim commands by finite difference, yaw wrap-aware and
// clamped to the gimbal's rate limit (a plate switch is a step: its raw
// difference is huge, the clamp keeps the feedforward term physical).
struct CommandRate {
    void reset() { has_ = false; }
    void update(double yaw, double pitch, double stamp, double max_rate, double& yaw_rate,
                double& pitch_rate);

private:
    bool has_ = false;
    double yaw_ = 0, pitch_ = 0, stamp_ = 0;
};

class RobotAimer {
public:
    RobotAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    AimCommand aim(const RobotTracker& tracker, double now) const;

private:
    int selectArmor(const RobotTracker& t, const RobotTracker::Vec& x, int previous,
                    double& facing) const;
    AimerConfig cfg_;
    const Ballistics& ball_;
    mutable int last_armor_ = -1;  // plate chosen last frame, for switch hysteresis
    mutable std::string last_id_;  // tracker id the hysteresis state belongs to
    mutable CommandRate rate_;     // B-01 feedforward state, same lifetime as last_id_
};

class PointAimer {
public:
    PointAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    AimCommand aim(const PointTracker& tracker, double now) const;

private:
    AimerConfig cfg_;
    const Ballistics& ball_;
    mutable CommandRate rate_;
};

class DirectAimer {
public:
    DirectAimer(const AimerConfig& cfg, const Ballistics& ballistics) : cfg_(cfg), ball_(ballistics) {}
    AimCommand aim(const FrameObservations& frame, double now) const;

private:
    AimerConfig cfg_;
    const Ballistics& ball_;
};

}  // namespace autoaim
