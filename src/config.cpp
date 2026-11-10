#include "autoaim/config.hpp"

#include <algorithm>
#include <cmath>

#include <stdexcept>

namespace autoaim {
namespace {

template <typename T>
void get(const cv::FileNode& n, const char* key, T& v) {
    const cv::FileNode child = n[key];
    if (!child.empty()) child >> v;
}
void get(const cv::FileNode& n, const char* key, float& v) {
    const cv::FileNode child = n[key];
    if (!child.empty()) v = static_cast<float>(static_cast<double>(child));
}
void get(const cv::FileNode& n, const char* key, bool& v) {
    const cv::FileNode child = n[key];
    if (!child.empty()) v = static_cast<int>(child) != 0;
}
void getStrings(const cv::FileNode& n, const char* key, std::vector<std::string>& v) {
    const cv::FileNode child = n[key];
    if (child.empty()) return;
    v.clear();
    for (const auto& item : child) v.push_back(static_cast<std::string>(item));
}
void getDoubles(const cv::FileNode& n, const char* key, std::vector<double>& v) {
    const cv::FileNode child = n[key];
    if (child.empty()) return;
    v.clear();
    for (const auto& item : child) v.push_back(static_cast<double>(item));
}

}  // namespace

Config Config::load(const std::string& path) {
    Config c;
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("cannot open config: " + path);

    const auto cam = fs["camera"];
    get(cam, "fx", c.camera.fx);
    get(cam, "fy", c.camera.fy);
    get(cam, "cx", c.camera.cx);
    get(cam, "cy", c.camera.cy);
    get(cam, "width", c.camera.width);
    get(cam, "height", c.camera.height);
    getDoubles(cam, "dist", c.camera.dist);
    get(cam, "pitch_deg", c.camera.pitch_deg);
    getDoubles(cam, "R_cam2gimbal", c.camera.R_cam2gimbal);
    getDoubles(cam, "t_cam2gimbal", c.camera.t_cam2gimbal);

    const auto d = fs["detector"];
    std::string color = "red";
    get(d, "enemy_color", color);
    c.detector.enemy_color = (color == "blue") ? Color::BLUE : Color::RED;
    get(d, "channel_diff_threshold", c.detector.channel_diff_threshold);
    { int v = 0; get(d, "channel_diff_adaptive", v); c.detector.channel_diff_adaptive = v != 0; }
    get(d, "channel_diff_k", c.detector.channel_diff_k);
    get(d, "channel_diff_min", c.detector.channel_diff_min);
    get(d, "channel_diff_max", c.detector.channel_diff_max);
    get(d, "min_light_length", c.detector.min_light_length);
    get(d, "min_light_aspect", c.detector.min_light_aspect);
    get(d, "max_light_aspect", c.detector.max_light_aspect);
    get(d, "min_fill_ratio", c.detector.min_fill_ratio);
    get(d, "max_light_tilt_deg", c.detector.max_light_tilt_deg);
    get(d, "max_pair_angle_diff_deg", c.detector.max_pair_angle_diff_deg);
    get(d, "max_length_ratio", c.detector.max_length_ratio);
    get(d, "max_center_y_diff", c.detector.max_center_y_diff);
    get(d, "min_gap_ratio", c.detector.min_gap_ratio);
    get(d, "max_gap_ratio", c.detector.max_gap_ratio);
    get(d, "min_plate_ratio", c.detector.min_plate_ratio);
    get(d, "max_plate_ratio", c.detector.max_plate_ratio);
    get(d, "large_plate_ratio", c.detector.large_plate_ratio);
    get(d, "reject_contained_lights", c.detector.reject_contained_lights);
    get(d, "refine_corners", c.detector.refine_corners);
    get(d, "model_path", c.detector.model_path);
    get(d, "label_path", c.detector.label_path);
    get(d, "classify_threshold", c.detector.classify_threshold);
    getStrings(d, "ignore_classes", c.detector.ignore_classes);
    get(d, "backend", c.detector.backend);
    get(d, "onnx_path", c.detector.onnx_path);
    getStrings(d, "onnx_classes", c.detector.onnx_classes);
    get(d, "keypoint_conf", c.detector.keypoint_conf);
    get(d, "keypoint_nms", c.detector.keypoint_nms);
    { int v = 640; get(d, "keypoint_input", v); c.detector.keypoint_input = v; }

    const auto p = fs["pose"];
    get(p, "optimize_yaw", c.pose.optimize_yaw);
    get(p, "armor_pitch_deg", c.pose.armor_pitch_deg);
    get(p, "yaw_search_range_deg", c.pose.yaw_search_range_deg);
    get(p, "yaw_gain", c.pose.yaw_gain);

    const auto t = fs["tracker"];
    get(t, "armors_num", c.tracker.armors_num);
    get(t, "max_match_distance", c.tracker.max_match_distance);
    get(t, "max_match_yaw_diff", c.tracker.max_match_yaw_diff);
    get(t, "tracking_thres", c.tracker.tracking_thres);
    get(t, "lost_time_thres", c.tracker.lost_time_thres);
    get(t, "q_xyz", c.tracker.q_xyz);
    get(t, "q_yaw", c.tracker.q_yaw);
    get(t, "q_r", c.tracker.q_r);
    get(t, "q_dz", c.tracker.q_dz);
    get(t, "pixel_noise", c.tracker.pixel_noise);
    get(t, "r_yaw", c.tracker.r_yaw);
    get(t, "r_yaw_head_on", c.tracker.r_yaw_head_on);
    get(t, "init_radius", c.tracker.init_radius);
    get(t, "min_radius", c.tracker.min_radius);
    get(t, "max_radius", c.tracker.max_radius);
    get(t, "q_adapt_max", c.tracker.q_adapt_max);
    get(t, "nis_ema_alpha", c.tracker.nis_ema_alpha);
    get(t, "singer_tau", c.tracker.singer_tau);
    { int v = 1; get(t, "carry_velocity", v); c.tracker.carry_velocity = v != 0; }
    get(t, "ca_model", c.tracker.ca_model);
    get(t, "q_acc", c.tracker.q_acc);

    const auto b = fs["ballistic"];
    get(b, "bullet_speed", c.ballistic.bullet_speed);
    get(b, "gravity", c.ballistic.gravity);
    get(b, "drag_k", c.ballistic.drag_k);
    get(b, "use_drag", c.ballistic.use_drag);
    { int v = 17; get(b, "bullet_mm", v); c.ballistic.bullet_mm = v; }
    get(b, "bullet_speed_42", c.ballistic.bullet_speed_42);
    get(b, "drag_k_42", c.ballistic.drag_k_42);

    const auto a = fs["aimer"];
    get(a, "latency", c.aimer.latency);
    get(a, "yaw_offset_deg", c.aimer.yaw_offset_deg);
    get(a, "pitch_offset_deg", c.aimer.pitch_offset_deg);
    get(a, "side_angle_deg", c.aimer.side_angle_deg);
    get(a, "min_switch_omega", c.aimer.min_switch_omega);
    get(a, "max_fire_facing_deg", c.aimer.max_fire_facing_deg);
    get(a, "center_mode_omega", c.aimer.center_mode_omega);
    get(a, "center_fire_window_deg", c.aimer.center_fire_window_deg);
    get(a, "fire_tolerance_scale", c.aimer.fire_tolerance_scale);
    get(a, "fire_sigma_scale", c.aimer.fire_sigma_scale);
    get(a, "gimbal_tau", c.aimer.gimbal_tau);
    get(a, "gimbal_max_rate", c.aimer.gimbal_max_rate);
    get(a, "gimbal_ff", c.aimer.gimbal_ff);
    get(a, "warmup_frames", c.aimer.warmup_frames);
    get(a, "switch_hysteresis_deg", c.aimer.switch_hysteresis_deg);

    const auto e = fs["eval"];
    get(e, "fps", c.eval.fps);
    getDoubles(e, "horizons", c.eval.horizons);
    const std::vector<std::string> problems = c.validate();
    if (!problems.empty()) {
        std::string msg = "invalid config " + path + ":";
        for (const auto& p : problems) msg += "\n  - " + p;
        throw std::invalid_argument(msg);
    }
    return c;
}

// Ported from the sentry review branch (docs/review_2026-11.md Â§4.9),
// adapted: this repo has no TargetConfig; per-class ballistic fields added.
std::vector<std::string> Config::validate() const {
    std::vector<std::string> e;
    auto need = [&e](bool ok, const std::string& what) {
        if (!ok) e.push_back(what);
    };
    need(camera.fx > 0 && camera.fy > 0, "camera.fx/fy must be > 0");
    need(camera.dist.size() <= 14, "camera.dist has more than 14 coefficients");
    need(camera.R_cam2gimbal.empty() || camera.R_cam2gimbal.size() == 9,
         "camera.R_cam2gimbal must be empty or have 9 values (row-major)");
    if (camera.R_cam2gimbal.size() == 9) {
        // must be a proper rotation: R R^T = I, det = +1
        const cv::Matx33d R(camera.R_cam2gimbal.data());
        const cv::Matx33d D = R * R.t() - cv::Matx33d::eye();
        double err = 0;
        for (int i = 0; i < 9; ++i) err = std::max(err, std::abs(D.val[i]));
        const double det = R(0, 0) * (R(1, 1) * R(2, 2) - R(1, 2) * R(2, 1)) -
                           R(0, 1) * (R(1, 0) * R(2, 2) - R(1, 2) * R(2, 0)) +
                           R(0, 2) * (R(1, 0) * R(2, 1) - R(1, 1) * R(2, 0));
        need(err < 1e-3 && det > 0, "camera.R_cam2gimbal is not a rotation matrix (orthonormal, det +1)");
    }
    need(camera.t_cam2gimbal.empty() || camera.t_cam2gimbal.size() == 3,
         "camera.t_cam2gimbal must be empty or have 3 values (metres)");
    need(detector.channel_diff_threshold >= 0 && detector.channel_diff_threshold < 255,
         "detector.channel_diff_threshold must be in [0, 255)");
    need(detector.classify_threshold >= 0 && detector.classify_threshold <= 1,
         "detector.classify_threshold must be in [0, 1]");
    need(tracker.armors_num >= 2 && tracker.armors_num <= 4, "tracker.armors_num must be 2, 3 or 4");
    need(tracker.min_radius > 0 && tracker.min_radius <= tracker.max_radius,
         "tracker.min_radius must be > 0 and <= max_radius");
    need(tracker.init_radius >= tracker.min_radius && tracker.init_radius <= tracker.max_radius,
         "tracker.init_radius must lie in [min_radius, max_radius]");
    need(tracker.tracking_thres >= 1, "tracker.tracking_thres must be >= 1");
    need(tracker.lost_time_thres > 0, "tracker.lost_time_thres must be > 0");
    need(tracker.singer_tau >= 0, "tracker.singer_tau must be >= 0 (0 = constant velocity)");
    need(tracker.nis_ema_alpha >= 0 && tracker.nis_ema_alpha <= 1, "tracker.nis_ema_alpha must be in [0, 1]");
    need(tracker.pixel_noise > 0 && tracker.r_yaw > 0, "tracker.pixel_noise and r_yaw must be > 0");
    need(tracker.q_xyz >= 0 && tracker.q_yaw >= 0 && tracker.q_r >= 0 && tracker.q_dz >= 0,
         "tracker process noises must be >= 0");
    need(ballistic.bullet_speed > 0, "ballistic.bullet_speed must be > 0");
    need(ballistic.gravity >= 0 && ballistic.drag_k >= 0, "ballistic.gravity and drag_k must be >= 0");
    need(aimer.latency >= 0 && aimer.latency < 1.0, "aimer.latency must be in [0, 1) s");
    need(aimer.gimbal_tau > 0 && aimer.gimbal_max_rate > 0, "aimer.gimbal_tau and gimbal_max_rate must be > 0");
    need(aimer.max_fire_facing_deg >= 0 && aimer.max_fire_facing_deg <= 90,
         "aimer.max_fire_facing_deg must be in [0, 90]");
    need(ballistic.bullet_mm == 17 || ballistic.bullet_mm == 42,
         "ballistic.bullet_mm must be 17 or 42");
    need(ballistic.bullet_speed_42 > 0 && ballistic.drag_k_42 >= 0,
         "ballistic.bullet_speed_42 must be > 0 and drag_k_42 >= 0");
    need(eval.fps > 0, "eval.fps must be > 0");
    return e;
}

bool setParam(Config& c, const std::string& key, double v) {
    // the parameters worth overriding / sweeping from the command line
    if (key == "fx") c.camera.fx = c.camera.fy = v;
    else if (key == "camera_pitch") c.camera.pitch_deg = v;
    else if (key == "latency") c.aimer.latency = v;
    else if (key == "bullet_speed") c.ballistic.bullet_speed = v;
    else if (key == "bullet_mm") c.ballistic.bullet_mm = static_cast<int>(std::lround(v));
    else if (key == "bullet_speed_42") c.ballistic.bullet_speed_42 = v;
    else if (key == "drag_k_42") c.ballistic.drag_k_42 = v;
    else if (key == "q_xyz") c.tracker.q_xyz = v;
    else if (key == "q_yaw") c.tracker.q_yaw = v;
    else if (key == "q_r") c.tracker.q_r = v;
    else if (key == "q_dz") c.tracker.q_dz = v;
    else if (key == "r_yaw") c.tracker.r_yaw = v;
    else if (key == "r_yaw_head_on") c.tracker.r_yaw_head_on = v;
    else if (key == "pixel_noise") c.tracker.pixel_noise = v;
    else if (key == "q_adapt_max") c.tracker.q_adapt_max = v;
    else if (key == "nis_alpha") c.tracker.nis_ema_alpha = v;
    else if (key == "singer_tau") c.tracker.singer_tau = v;
    else if (key == "carry_velocity") c.tracker.carry_velocity = v != 0;
    else if (key == "channel_diff_threshold") c.detector.channel_diff_threshold = static_cast<int>(std::lround(v));
    else if (key == "diff_adaptive") c.detector.channel_diff_adaptive = v != 0;
    else if (key == "ca_model") c.tracker.ca_model = v != 0;
    else if (key == "q_acc") c.tracker.q_acc = v;
    else if (key == "max_match_distance") c.tracker.max_match_distance = v;
    else if (key == "side_angle") c.aimer.side_angle_deg = v;
    else if (key == "max_fire_facing") c.aimer.max_fire_facing_deg = v;
    else if (key == "center_mode_omega") c.aimer.center_mode_omega = v;
    else if (key == "center_fire_window") c.aimer.center_fire_window_deg = v;
    else if (key == "fire_tolerance") c.aimer.fire_tolerance_scale = v;
    else if (key == "fire_sigma") c.aimer.fire_sigma_scale = v;
    else if (key == "gimbal_tau") c.aimer.gimbal_tau = v;
    else if (key == "gimbal_ff") c.aimer.gimbal_ff = v;
    else if (key == "warmup_frames") c.aimer.warmup_frames = static_cast<int>(std::lround(v));
    else if (key == "switch_hysteresis") c.aimer.switch_hysteresis_deg = v;
    else if (key == "armor_pitch") c.pose.armor_pitch_deg = v;
    else if (key == "optimize_yaw") c.pose.optimize_yaw = v != 0;
    else if (key == "yaw_gain") c.pose.yaw_gain = v;
    else if (key == "refine_corners") c.detector.refine_corners = v != 0;
    else if (key == "diff_adaptive") c.detector.channel_diff_adaptive = v != 0;
    else if (key == "diff_k") c.detector.channel_diff_k = v;
    else if (key == "diff_min") c.detector.channel_diff_min = v;
    else if (key == "diff_max") c.detector.channel_diff_max = v;
    else if (key == "use_drag") c.ballistic.use_drag = v != 0;
    else return false;
    return true;
}

void Config::save(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    fs << "camera" << "{"
       << "fx" << camera.fx << "fy" << camera.fy << "cx" << camera.cx << "cy" << camera.cy
       << "dist" << camera.dist << "pitch_deg" << camera.pitch_deg << "}";
    fs << "tracker" << "{"
       << "q_xyz" << tracker.q_xyz << "q_yaw" << tracker.q_yaw << "r_yaw" << tracker.r_yaw
       << "r_yaw_head_on" << tracker.r_yaw_head_on << "pixel_noise" << tracker.pixel_noise
       << "max_match_distance" << tracker.max_match_distance << "}";
    fs << "aimer" << "{"
       << "latency" << aimer.latency << "side_angle_deg" << aimer.side_angle_deg
       << "center_mode_omega" << aimer.center_mode_omega << "}";
}

}  // namespace autoaim
