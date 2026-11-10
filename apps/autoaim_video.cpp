// Run the full auto-aim pipeline on a recorded video and evaluate it.
//
//   autoaim_video <video> [--config cfg.yaml] [--out dir] [--redetect]
//                 [--render A:B] [--fx F] [--latency S] [--set key=value ...]
//                 [--sweep name=v1,v2,...]
//
// Pass 1 (cached): detector on every frame -> <out>/<stem>_detections.txt
// Pass 2: pose + trackers + aimers + virtual shooting -> report / CSV.
// --render A:B writes an overlay video of frames A..B.
// --sweep re-runs pass 2 for each value (detections are reused).
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "autoaim/config.hpp"
#include "autoaim/detector.hpp"
#include "autoaim/draw.hpp"
#include "autoaim/evaluation.hpp"
#include "autoaim/pose.hpp"

using namespace autoaim;
namespace fs = std::filesystem;

namespace {

// A non-cryptographic content fingerprint is sufficient for cache invalidation.
// Hash the complete video and classifier assets so two files with the same stem
// and frame count cannot silently share detections.
std::uint64_t fileHash(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot hash input: " + path);
    std::uint64_t h = 14695981039346656037ULL;
    char buf[1 << 16];
    while (in.read(buf, sizeof(buf)) || in.gcount() > 0)
        for (std::streamsize i = 0; i < in.gcount(); ++i) {
            h ^= static_cast<unsigned char>(buf[i]);
            h *= 1099511628211ULL;
        }
    if (in.bad()) throw std::runtime_error("cannot finish hashing input: " + path);
    return h;
}

std::string cacheSignature(const std::string& video, const DetectorConfig& d) {
    std::ostringstream s;
    s << std::setprecision(17) << "detector-v2 " << CV_VERSION << ' ' << fileHash(video) << ' '
      << fileHash(d.model_path) << ' ' << fileHash(d.label_path) << ' '
      << static_cast<int>(d.enemy_color) << ' ' << d.channel_diff_threshold << ' '
      << d.min_light_length << ' ' << d.min_light_aspect << ' ' << d.max_light_aspect << ' '
      << d.min_fill_ratio << ' ' << d.max_light_tilt_deg << ' ' << d.max_pair_angle_diff_deg << ' '
      << d.max_length_ratio << ' ' << d.max_center_y_diff << ' ' << d.min_gap_ratio << ' '
      << d.max_gap_ratio << ' ' << d.min_plate_ratio << ' ' << d.max_plate_ratio << ' '
      << d.large_plate_ratio << ' ' << d.reject_contained_lights << ' ' << d.refine_corners << ' '
      << d.classify_threshold;
    for (const auto& name : d.ignore_classes) s << ' ' << name.size() << ':' << name;
    std::uint64_t h = 14695981039346656037ULL;
    for (unsigned char c : s.str()) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    std::ostringstream hex;
    hex << std::hex << std::setw(16) << std::setfill('0') << h;
    return hex.str();
}

bool cacheMetadataMatches(const std::string& path, const std::string& signature,
                          std::size_t frames) {
    try {
        cv::FileStorage meta(path, cv::FileStorage::READ);
        if (!meta.isOpened() || meta["version"].empty() || meta["signature"].empty() ||
            meta["frames"].empty()) return false;
        return static_cast<int>(meta["version"]) == 2 &&
               static_cast<std::string>(meta["signature"]) == signature &&
               static_cast<int>(meta["frames"]) == static_cast<int>(frames);
    } catch (const cv::Exception&) {
        return false;
    }
}

void saveCacheMetadata(const std::string& path, const std::string& signature,
                       std::size_t frames) {
    cv::FileStorage meta(path, cv::FileStorage::WRITE);
    if (!meta.isOpened()) throw std::runtime_error("cannot write cache metadata: " + path);
    meta << "version" << 2 << "signature" << signature << "frames" << static_cast<int>(frames);
}

std::vector<FrameDetections> runDetector(const std::string& video, const Config& cfg, double& ms_per_frame) {
    cv::VideoCapture cap(video);
    if (!cap.isOpened()) throw std::runtime_error("cannot open video " + video);
    Detector det(cfg.detector);
    std::vector<FrameDetections> out;
    cv::Mat frame;
    int n = 0;
    double total = 0;
    while (cap.read(frame)) {
        const auto t0 = std::chrono::steady_clock::now();
        FrameDetections f;
        f.frame = n;
        f.stamp = n / cfg.eval.fps;
        f.armors = det.detect(frame);
        total += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        out.push_back(std::move(f));
        if (++n % 200 == 0) std::cerr << "  detect " << n << " frames\r";
    }
    std::cerr << "\n";
    ms_per_frame = n ? total / n : 0;
    return out;
}

void writeCsv(const std::string& path, const EvalResult& r) {
    std::ofstream o(path);
    o << "frame,state,xc,yc,zc,vxc,vyc,yaw,w,r1,r2,dz,matched,cmd_valid,center_mode,armor_k,facing_deg,"
         "cmd_yaw_deg,cmd_pitch_deg,t_fly,gimbal_yaw_deg,gimbal_pitch_deg,shot,shot_result,miss_px,off_u,off_v\n";
    for (const auto& l : r.log) {
        const auto& x = l.x;
        o << l.frame << ',' << toString(l.state) << ',' << x[0] << ',' << x[2] << ',' << x[4] << ',' << x[1] << ','
          << x[3] << ',' << x[6] << ',' << x[7] << ',' << x[8] << ',' << x[9] << ',' << x[10] << ',' << l.matched
          << ',' << l.cmd.valid << ',' << l.cmd.center_mode << ',' << l.cmd.armor_k << ',' << l.cmd.facing_deg
          << ',' << l.cmd.yaw * 180 / kPi << ',' << l.cmd.pitch * 180 / kPi << ',' << l.cmd.t_fly << ','
          << l.gimbal_yaw * 180 / kPi << ',' << l.gimbal_pitch * 180 / kPi << ',' << l.shot << ','
          << static_cast<int>(l.shot_rec.result) << ',' << l.shot_rec.miss_px << ',' << l.shot_rec.off_u << ','
          << l.shot_rec.off_v << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: autoaim_video <video> [--config cfg.yaml] [--out dir] [--redetect] [--render A:B]\n"
                     "                     [--camera calibrated.yml] [--set key=value]... [--sweep key=v1,v2,...]\n";
        return 1;
    }
    const std::string video = argv[1];
    std::string config_path = std::string(AUTOAIM_ROOT) + "/config/default.yaml";
    std::string camera_path;
    std::string out_dir = "out";
    bool redetect = false;
    int render_a = -1, render_b = -1;
    std::vector<std::pair<std::string, double>> sets;
    std::string sweep_key;
    std::vector<double> sweep_vals;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (a == "--camera" && i + 1 < argc) camera_path = argv[++i];
        else if (a == "--out" && i + 1 < argc) out_dir = argv[++i];
        else if (a == "--redetect") redetect = true;
        else if (a == "--render" && i + 1 < argc) std::sscanf(argv[++i], "%d:%d", &render_a, &render_b);
        else if (a == "--set" && i + 1 < argc) {
            const std::string kv = argv[++i];
            const auto eq = kv.find('=');
            sets.emplace_back(kv.substr(0, eq), std::stod(kv.substr(eq + 1)));
        } else if (a == "--sweep" && i + 1 < argc) {
            const std::string kv = argv[++i];
            const auto eq = kv.find('=');
            sweep_key = kv.substr(0, eq);
            std::stringstream ss(kv.substr(eq + 1));
            std::string tok;
            while (std::getline(ss, tok, ',')) sweep_vals.push_back(std::stod(tok));
        } else {
            std::cerr << "unknown argument " << a << '\n';
            return 1;
        }
    }

    Config cfg = Config::load(config_path);
    cv::VideoCapture probe(video);
    if (!probe.isOpened()) {
        std::cerr << "cannot open video " << video << '\n';
        return 1;
    }
    const double recorded_fps = probe.get(cv::CAP_PROP_FPS);
    if (std::isfinite(recorded_fps) && recorded_fps >= 1.0 && recorded_fps <= 1000.0)
        cfg.eval.fps = recorded_fps;
    const double recorded_frames = probe.get(cv::CAP_PROP_FRAME_COUNT);
    if (!camera_path.empty()) {
        cv::FileStorage camera_file(camera_path, cv::FileStorage::READ);
        const cv::FileNode camera_node = camera_file["camera"];
        for (const char* key : {"fx", "fy", "cx", "cy", "dist", "width", "height"}) {
            if (camera_node.empty() || camera_node[key].empty()) {
                std::cerr << "camera file missing camera." << key << '\n';
                return 1;
            }
        }
        cfg.camera = Config::load(camera_path).camera;
        if (cfg.camera.width <= 0 || cfg.camera.height <= 0 || !std::isfinite(cfg.camera.fx) ||
            !std::isfinite(cfg.camera.fy) || cfg.camera.fx <= 0 || cfg.camera.fy <= 0 ||
            !std::isfinite(cfg.camera.cx) || !std::isfinite(cfg.camera.cy) ||
            cfg.camera.dist.size() < 4) {
            std::cerr << "camera file contains invalid calibration values\n";
            return 1;
        }
        for (double v : cfg.camera.dist)
            if (!std::isfinite(v)) {
                std::cerr << "camera file contains invalid distortion coefficients\n";
                return 1;
            }
        const int width = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_WIDTH));
        const int height = static_cast<int>(probe.get(cv::CAP_PROP_FRAME_HEIGHT));
        if (width != cfg.camera.width || height != cfg.camera.height) {
            std::cerr << "camera calibration size " << cfg.camera.width << 'x' << cfg.camera.height
                      << " does not match video " << width << 'x' << height << '\n';
            return 1;
        }
    }
    if (cfg.detector.model_path.empty()) cfg.detector.model_path = std::string(AUTOAIM_ROOT) + "/model/mlp.onnx";
    if (cfg.detector.label_path.empty()) cfg.detector.label_path = std::string(AUTOAIM_ROOT) + "/model/label.txt";
    for (const auto& [k, v] : sets)
        if (!setParam(cfg, k, v)) {
            std::cerr << "unknown parameter " << k << '\n';
            return 1;
        }
    if (sweep_key == "refine_corners") {
        std::cerr << "refine_corners changes detections; use separate --set runs instead of --sweep\n";
        return 1;
    }

    fs::create_directories(out_dir);
    const std::string stem = fs::path(video).stem().string();
    const std::string cache = out_dir + "/" + stem + "_detections.txt";
    const std::string metadata = cache + ".meta.yml";
    const std::string signature = cacheSignature(video, cfg.detector);

    std::vector<FrameDetections> dets;
    double det_ms = 0;
    const bool cache_ok = !redetect && loadDetections(cache, dets) && !dets.empty() &&
        cacheMetadataMatches(metadata, signature, dets.size()) &&
        (!std::isfinite(recorded_frames) || recorded_frames < 1.0 ||
         dets.size() == static_cast<std::size_t>(std::llround(recorded_frames)));
    if (cache_ok) {
        // The video's actual rate owns the replay timeline; detections are pixels.
        for (auto& f : dets) f.stamp = f.frame / cfg.eval.fps;
        std::cout << "loaded " << dets.size() << " frames of detections from " << cache << '\n';
    } else {
        fs::remove(metadata);
        dets = runDetector(video, cfg, det_ms);
        if (!saveDetections(cache, dets)) throw std::runtime_error("cannot save detections: " + cache);
        saveCacheMetadata(metadata, signature, dets.size());
        std::cout << "detected " << dets.size() << " frames, " << det_ms << " ms/frame -> " << cache << '\n';
    }

    if (!sweep_key.empty()) {
        std::cout << "| " << sweep_key
                  << " | robot hit% | robot hits/s | point hit% | direct hit% | oracle hit% | pred robot 0.2s | "
                     "pred point 0.2s | median r1 | median |w| |\n|---|---|---|---|---|---|---|---|---|---|\n";
        for (double v : sweep_vals) {
            Config c = cfg;
            if (!setParam(c, sweep_key, v)) {
                std::cerr << "unknown parameter " << sweep_key << '\n';
                return 1;
            }
            CameraModel cam(c.camera);
            PoseSolver solver(cam, c.pose);
            const auto obs = computeObservations(dets, solver);
            const auto r = evaluate(c, obs, cam, false);
            double pr = 0, pp = 0;
            for (const auto& p : r.predictions)
                if (std::abs(p.horizon - 0.2) < 1e-6) {
                    if (p.name == "robot EKF") pr = p.median_px;
                    if (p.name == "point KF") pp = p.median_px;
                }
            std::printf("| %g | %.1f | %.2f | %.1f | %.1f | %.1f | %.1f | %.1f | %.3f | %.2f |\n", v,
                        100 * r.strategies[2].hit_rate(), r.strategies[2].hits_per_second(c.eval.fps),
                        100 * r.strategies[1].hit_rate(), 100 * r.strategies[0].hit_rate(),
                        100 * r.strategies[3].hit_rate(), pr, pp, r.median_r1, r.median_abs_omega);
        }
        return 0;
    }

    CameraModel camera(cfg.camera);
    PoseSolver solver(camera, cfg.pose);
    const auto t0 = std::chrono::steady_clock::now();
    const auto obs = computeObservations(dets, solver);
    const auto t1 = std::chrono::steady_clock::now();
    const EvalResult res = evaluate(cfg, obs, camera, true);
    const auto t2 = std::chrono::steady_clock::now();
    const double pose_ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / std::max<size_t>(1, obs.size());
    const double eval_ms = std::chrono::duration<double, std::milli>(t2 - t1).count() / std::max<size_t>(1, obs.size());

    const std::string report = formatReport(res, cfg.eval.fps);
    std::cout << "\n" << report;
    std::printf("timing: pose %.3f ms/frame, trackers+aimers+judge %.3f ms/frame%s\n", pose_ms, eval_ms,
                det_ms > 0 ? (", detector " + std::to_string(det_ms) + " ms/frame").c_str() : "");
    {
        std::ofstream o(out_dir + "/" + stem + "_report.md");
        o << "# " << stem << "\n\n" << report;
    }
    writeCsv(out_dir + "/" + stem + "_track.csv", res);

    // Yaw diagnostic: measured plate yaw vs the yaw implied by the plate's
    // lateral offset from the estimated robot centre (lateral position is the
    // best-measured quantity; a slope != 1 here means a biased yaw measurement).
    {
        std::ofstream o(out_dir + "/" + stem + "_yawdiag.csv");
        o << "frame,alpha_meas_deg,alpha_geo_deg,distance,bar_px,reproj\n";
        for (std::size_t n = 0; n < obs.size(); ++n) {
            const auto& l = res.log[n];
            if (l.state != TrackState::TRACKING) continue;
            const double xc = l.x[RobotTracker::XC], yc = l.x[RobotTracker::YC];
            const double los = std::atan2(yc, xc);
            const double r = 0.5 * (l.x[RobotTracker::R1] + l.x[RobotTracker::R2]);
            const cv::Vec2d nrm(-std::sin(los), std::cos(los));
            for (const auto& a : obs[n].armors) {
                const double lat = (a.position[0] - xc) * nrm[0] + (a.position[1] - yc) * nrm[1];
                const double s = std::clamp(-lat / r, -1.0, 1.0);
                o << n << ',' << wrapAngle(a.yaw - los) * 180 / kPi << ',' << std::asin(s) * 180 / kPi << ','
                  << a.distance << ',' << 0.5 * (a.det.left.length + a.det.right.length) << ',' << a.reproj_error
                  << '\n';
            }
        }
        // Model-free check: two plates of one robot seen together are 90 deg apart.
        std::ofstream o2(out_dir + "/" + stem + "_pairdiag.csv");
        o2 << "frame,yaw_diff_deg,width_ratio_a,width_ratio_b,bar_px_a,bar_px_b\n";
        for (std::size_t n = 0; n < obs.size(); ++n) {
            const auto& A = obs[n].armors;
            if (A.size() != 2 || deviceId(A[0].det) != deviceId(A[1].det)) continue;
            auto wr = [](const ArmorObservation& a) {
                const double w = 0.5 * (cv::norm(a.det.corners[3] - a.det.corners[0]) + cv::norm(a.det.corners[2] - a.det.corners[1]));
                const double h = 0.5 * (a.det.left.length + a.det.right.length);
                return w / h;
            };
            o2 << n << ',' << std::abs(wrapAngle(A[0].yaw - A[1].yaw)) * 180 / kPi << ',' << wr(A[0]) << ',' << wr(A[1])
               << ',' << 0.5 * (A[0].det.left.length + A[0].det.right.length) << ','
               << 0.5 * (A[1].det.left.length + A[1].det.right.length) << '\n';
        }
    }

    if (render_a >= 0 && render_b > render_a) {
        cv::VideoCapture cap(video);
        cap.set(cv::CAP_PROP_POS_FRAMES, render_a);
        RobotTracker geom(cfg.tracker, camera);
        cv::VideoWriter writer;
        const std::string vid_path = out_dir + "/" + stem + "_overlay_" + std::to_string(render_a) + "_" +
                                     std::to_string(render_b) + ".avi";
        cv::Mat frame;
        for (int n = render_a; n <= render_b && n < static_cast<int>(obs.size()) && cap.read(frame); ++n) {
            drawOverlay(frame, obs[n], res.log[n], geom, camera);
            cv::Mat small;
            cv::resize(frame, small, cv::Size(), 0.75, 0.75, cv::INTER_AREA);
            if (!writer.isOpened())
                writer.open(vid_path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), cfg.eval.fps, small.size());
            writer.write(small);
            if ((n - render_a) % 25 == 0) {
                char name[256];
                std::snprintf(name, sizeof(name), "%s/%s_overlay_%05d.png", out_dir.c_str(), stem.c_str(), n);
                cv::imwrite(name, frame);
            }
        }
        std::cout << "overlay -> " << vid_path << '\n';
    }
    return 0;
}
