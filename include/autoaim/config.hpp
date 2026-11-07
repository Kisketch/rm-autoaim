// All tunable parameters, loaded from a YAML file (cv::FileStorage).
// Every field has a default so a missing key never breaks a run.
#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

#include "autoaim/types.hpp"

namespace autoaim {

struct CameraConfig {
    double fx = 2400, fy = 2400, cx = 720, cy = 540;
    int width = 0, height = 0;  // 0 for legacy configs without a declared image size
    std::vector<double> dist = {0, 0, 0, 0, 0};
    double pitch_deg = 0.0;   // camera tilt, positive = looking down
    cv::Matx33d K() const { return {fx, 0, cx, 0, fy, cy, 0, 0, 1}; }
};

struct DetectorConfig {
    Color enemy_color = Color::RED;
    int channel_diff_threshold = 70;  // R-B (red) / B-R (blue) binarisation threshold
    // Adaptive variant: thr = clamp(mean + k*std, min, max) computed per frame.
    // Fixes the collapse under dim light measured in docs/exposure_robustness.md.
    bool channel_diff_adaptive = false;
    double channel_diff_k = 4.5;
    double channel_diff_min = 40.0;
    double channel_diff_max = 120.0;
    // light bar filters
    float min_light_length = 8.f;
    float min_light_aspect = 1.6f, max_light_aspect = 12.f;
    float min_fill_ratio = 0.40f;
    float max_light_tilt_deg = 40.f;
    // light pairing
    float max_pair_angle_diff_deg = 12.f;
    float max_length_ratio = 1.7f;
    float max_center_y_diff = 0.6f;       // / mean bar length
    float min_gap_ratio = 0.15f, max_gap_ratio = 3.2f;
    float min_plate_ratio = 1.1f, max_plate_ratio = 5.0f;
    float large_plate_ratio = 3.0f;       // centre distance / bar length above which -> LARGE
    bool reject_contained_lights = true;  // rm_auto_aim: no other light inside the pair box
    bool refine_corners = true;           // FYT2024: PCA symmetry-axis corner refinement
    // number classifier
    std::string model_path, label_path;
    double classify_threshold = 0.7;
    std::vector<std::string> ignore_classes = {"negative"};
};

struct PoseConfig {
    bool optimize_yaw = true;       // 1-D yaw search with fixed armor pitch
    double armor_pitch_deg = 15.0;  // RoboMaster armors lean back 15 degrees
    double yaw_search_range_deg = 85.0;
    // Gain on the plate angle relative to the line of sight. Bar glow makes
    // oblique plates look too wide, so image-based yaw is biased toward 0.
    // Self-calibrated from frames that show two plates of one robot (they must
    // be 90 deg apart): yaw_gain = 90 / measured separation. See README.
    double yaw_gain = 1.0;
};

struct TrackerConfig {
    int armors_num = 4;               // 4 standard, 2 balance, 3 outpost
    double max_match_distance = 0.35; // m, association gate
    double max_match_yaw_diff = 1.0;  // rad
    int tracking_thres = 3;           // consecutive matches DETECTING -> TRACKING
    double lost_time_thres = 0.4;     // s in TEMP_LOST before LOST
    // process noise (continuous white-noise acceleration spectral densities)
    double q_xyz = 4.0;
    double q_yaw = 30.0;
    double q_r = 1e-4;
    double q_dz = 1e-4;
    // measurement noise
    double pixel_noise = 1.0;         // px, corner noise used for anisotropic position R
    double r_yaw = 0.06;              // rad^2
    double r_yaw_head_on = 0.25;      // extra yaw variance when the plate faces the camera
    double init_radius = 0.25;
    double min_radius = 0.12, max_radius = 0.40;
    // Adaptive process noise: q_xyz is multiplied by clamp(NIS_ema / 3, 1, q_adapt_max)
    // (position-innovation NIS, 3 dof). Stiff while spinning in place, loose when
    // the target manoeuvres. 1 disables the adaptation.
    double q_adapt_max = 20.0;
    double nis_ema_alpha = 0.2;
    // Singer velocity model: the target's velocity is damped with time constant
    // tau instead of being held constant. 0 keeps the constant-velocity model.
    // Rationale: a strafing robot reverses direction within the 0.3 s lead, and
    // a CV extrapolation overshoots; tau ~ 1 s matches a 2 s strafe period.
    double singer_tau = 0.0;
    // Carry vx/vy/omega through a short LOST: the plate jumps, the robot keeps moving.
    bool carry_velocity = true;
    // Constant-acceleration extension for the robot centre: two extra states
    // (AXC, AYC) appended after DZ so no existing index moves. q_acc is the
    // jerk spectral density; 0 disables the model.
    double ca_model = 0.0;
    double q_acc = 2.0;
};

struct BallisticConfig {
    double bullet_speed = 25.0;  // m/s
    double gravity = 9.8;
    // quadratic drag: a = -k |v| v,  k = 0.5 rho Cd A / m
    // 17 mm: m = 3.2 g, d = 16.8 mm, Cd ~ 0.47, rho = 1.169 -> k ~ 0.019 1/m
    double drag_k = 0.019;
    bool use_drag = true;
};

struct AimerConfig {
    double latency = 0.06;           // s, capture -> bullet leaves the barrel
    double side_angle_deg = 20.0;    // switch early to the incoming armor (FYT2024)
    // Keep the current plate unless the next one is better by this many degrees.
    // Stops the selector from dithering around the switch boundary.
    double switch_hysteresis_deg = 0.0;
    double min_switch_omega = 1.0;   // rad/s, below this no early switch
    double max_fire_facing_deg = 50; // do not fire at plates turned further away than this
    double center_mode_omega = 6.0;  // rad/s, above this aim at the robot centre
    double center_fire_window_deg = 15.0;  // upper bound; the window also follows the radius
    double fire_tolerance_scale = 0.9;  // fraction of the plate half-size used as tolerance
    // Fire only if the predicted lateral std of the aimed plate at impact time is
    // below this fraction of the panel half-width (0 disables the gate).
    double fire_sigma_scale = 0.6;
    // simulated gimbal
    double gimbal_tau = 0.03;        // s, first-order lag
    double gimbal_max_rate = 12.0;
    // Do not fire during the first N frames after (re)acquiring TRACKING.
    int warmup_frames = 0;       // frames since (re)acquiring TRACKING
    // Velocity feedforward into the simulated gimbal (0 = pure P lag).
    // Models a real gimbal that also receives the commanded yaw rate.
    double gimbal_ff = 0.0;
};

struct EvalConfig {
    double fps = 25.0;
    std::vector<double> horizons = {0.12, 0.20, 0.32};  // s, prediction-error horizons
};

struct Config {
    CameraConfig camera;
    DetectorConfig detector;
    PoseConfig pose;
    TrackerConfig tracker;
    BallisticConfig ballistic;
    AimerConfig aimer;
    EvalConfig eval;

    // Load from YAML; keys that are missing keep their defaults.
    static Config load(const std::string& path);
    void save(const std::string& path) const;
};

// Command-line override of a named parameter (see config.cpp for the names).
bool setParam(Config& c, const std::string& key, double v);

}  // namespace autoaim
