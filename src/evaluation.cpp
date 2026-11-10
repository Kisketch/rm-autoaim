#include "autoaim/evaluation.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

namespace autoaim {

// ------------------------------------------------------------ detection cache

bool saveDetections(const std::string& path, const std::vector<FrameDetections>& frames) {
    std::ofstream out(path);
    if (!out) return false;
    // Round-trip the float endpoints exactly. The old millipixel rounding
    // (fixed << setprecision(3)) changed boundary shots on replay even with the
    // same video and config.
    out << std::setprecision(17);
    for (const auto& f : frames) {
        out << f.frame << ' ' << f.stamp << ' ' << f.armors.size();
        for (const auto& a : f.armors) {
            out << ' ' << a.number << ' ' << a.confidence << ' ' << static_cast<int>(a.type) << ' '
                << a.left.top.x << ' ' << a.left.top.y << ' ' << a.left.bottom.x << ' '
                << a.left.bottom.y << ' ' << a.left.width << ' ' << a.right.top.x << ' '
                << a.right.top.y << ' ' << a.right.bottom.x << ' ' << a.right.bottom.y << ' '
                << a.right.width;
        }
        out << '\n';
    }
    return static_cast<bool>(out);
}

namespace {
Light lightFrom(const cv::Point2f& top, const cv::Point2f& bottom, float width) {
    Light l;
    l.top = top, l.bottom = bottom, l.center = (top + bottom) * 0.5f;
    l.length = static_cast<float>(cv::norm(top - bottom));
    l.width = width;
    l.tilt_deg = static_cast<float>(std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y)) * 180 / kPi);
    l.rect = cv::RotatedRect(l.center, cv::Size2f(width, l.length), 0);
    return l;
}
}  // namespace

bool loadDetections(const std::string& path, std::vector<FrameDetections>& frames) {
    std::ifstream in(path);
    if (!in) return false;
    frames.clear();
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        FrameDetections f;
        std::size_t n = 0;
        if (!(ss >> f.frame >> f.stamp >> n) || f.frame != static_cast<int>(frames.size()) ||
            !std::isfinite(f.stamp) || n > 128) return false;
        for (std::size_t i = 0; i < n; ++i) {
            ArmorDetection a;
            int type = 0;
            cv::Point2f lt, lb, rt, rb;
            float lw, rw;
            if (!(ss >> a.number >> a.confidence >> type >> lt.x >> lt.y >> lb.x >> lb.y >> lw >>
                  rt.x >> rt.y >> rb.x >> rb.y >> rw) ||
                (type != 0 && type != 1) || !std::isfinite(a.confidence) ||
                !std::isfinite(lt.x) || !std::isfinite(lt.y) ||
                !std::isfinite(lb.x) || !std::isfinite(lb.y) ||
                !std::isfinite(rt.x) || !std::isfinite(rt.y) ||
                !std::isfinite(rb.x) || !std::isfinite(rb.y) ||
                !std::isfinite(lw) || !std::isfinite(rw) || lw <= 0 || rw <= 0) return false;
            a.type = static_cast<ArmorType>(type);
            a.left = lightFrom(lt, lb, lw);
            a.right = lightFrom(rt, rb, rw);
            a.corners = {lb, lt, rt, rb};
            a.center = (lb + lt + rt + rb) * 0.25f;
            f.armors.push_back(a);
        }
        std::string extra;
        if (ss >> extra) return false;  // trailing garbage: do not trust the line
        frames.push_back(f);
    }
    return !in.bad() && !frames.empty();
}

std::vector<FrameObservations> computeObservations(const std::vector<FrameDetections>& frames,
                                                   const PoseSolver& solver) {
    std::vector<FrameObservations> out;
    out.reserve(frames.size());
    for (const auto& f : frames) {
        FrameObservations o;
        o.frame = f.frame;
        o.stamp = f.stamp;
        o.armors = solver.solveAll(f.armors);
        out.push_back(std::move(o));
    }
    return out;
}

std::vector<cv::Point2f> panelPolygon(const ArmorDetection& det) {
    const ArmorGeometry& g = geometryOf(det.type);
    const float sv = static_cast<float>(g.panel_height / g.light_height);
    const float sh = static_cast<float>(g.panel_width / g.light_width);
    const cv::Point2f lm = (det.left.top + det.left.bottom) * 0.5f;
    const cv::Point2f rm = (det.right.top + det.right.bottom) * 0.5f;
    const cv::Point2f lh = (det.left.top - det.left.bottom) * 0.5f * sv;
    const cv::Point2f rh = (det.right.top - det.right.bottom) * 0.5f * sv;
    const cv::Point2f c = (lm + rm) * 0.5f;
    const cv::Point2f lm2 = c + (lm - c) * sh, rm2 = c + (rm - c) * sh;
    return {lm2 - lh, lm2 + lh, rm2 + rh, rm2 - rh};
}

// ------------------------------------------------------------ shot judge

namespace {
ArmorObservation lerpObservation(const ArmorObservation& a, const ArmorObservation& b, float w) {
    ArmorObservation o = a;
    auto lp = [w](const cv::Point2f& p, const cv::Point2f& q) { return p * (1 - w) + q * w; };
    o.position = a.position * (1 - w) + b.position * w;
    o.det.left.top = lp(a.det.left.top, b.det.left.top);
    o.det.left.bottom = lp(a.det.left.bottom, b.det.left.bottom);
    o.det.right.top = lp(a.det.right.top, b.det.right.top);
    o.det.right.bottom = lp(a.det.right.bottom, b.det.right.bottom);
    for (int i = 0; i < 4; ++i) o.det.corners[i] = lp(a.det.corners[i], b.det.corners[i]);
    o.det.center = lp(a.det.center, b.det.center);
    return o;
}
}  // namespace

// Plates at an arbitrary time T: the two frames around T are interpolated, plate
// by plate (nearest centre). Plates seen in only one of the two frames are used
// when T is closer to that frame. Returns false if T is outside the recording.
bool ShotJudge::platesAt(double T, const std::string& id, std::vector<ArmorObservation>& out) const {
    out.clear();
    const double ft = T * fps_;
    const int f0 = static_cast<int>(std::floor(ft));
    const int f1 = f0 + 1;
    if (f0 < 0 || f1 >= static_cast<int>(obs_.size())) return false;
    const float w = static_cast<float>(ft - f0);
    const auto& A = obs_[f0].armors;
    const auto& B = obs_[f1].armors;
    std::vector<bool> used_b(B.size(), false);
    for (const auto& a : A) {
        if (!id.empty() && deviceId(a.det) != id) continue;
        int best = -1;
        double best_d = 1e18;
        const double gate = 0.6 * cv::norm(a.det.corners[3] - a.det.corners[0]) + 3.0;
        for (int j = 0; j < static_cast<int>(B.size()); ++j) {
            if (used_b[j] || deviceId(B[j].det) != deviceId(a.det)) continue;
            const double d = cv::norm(B[j].det.center - a.det.center);
            if (d < gate && d < best_d) best_d = d, best = j;
        }
        if (best >= 0) {
            used_b[best] = true;
            out.push_back(lerpObservation(a, B[best], w));
        } else if (w < 0.5f) {
            out.push_back(a);
        }
    }
    if (w >= 0.5f)
        for (int j = 0; j < static_cast<int>(B.size()); ++j)
            if (!used_b[j] && (id.empty() || deviceId(B[j].det) == id)) out.push_back(B[j]);
    return true;
}

ShotResult ShotJudge::judge(double t_depart, double yaw, double pitch, double t_fly_hint,
                            const std::string& id, ShotRecord& rec) const {
    double* miss_px = &rec.miss_px;
    int* impact_frame = &rec.impact_frame;
    cv::Point2d* impact_px = &rec.impact_px;
    std::vector<ArmorObservation> plates;
    // time of flight depends on which plate the bullet meets; refine once
    double T = t_depart + t_fly_hint;
    if (!platesAt(T, id, plates)) return ShotResult::UNKNOWN;
    if (plates.empty()) return ShotResult::UNKNOWN;
    {
        double best_s = 1e18, h, t;
        for (const auto& a : plates) best_s = std::min(best_s, std::hypot(a.position[0], a.position[1]));
        if (ball_.simulate(pitch, best_s, h, t)) {
            T = t_depart + t;
            if (!platesAt(T, id, plates) || plates.empty()) return ShotResult::UNKNOWN;
        }
    }
    if (impact_frame) *impact_frame = static_cast<int>(std::lround(T * fps_));
    double best_miss = 1e18, best_center_d = 1e18;
    ShotResult result = ShotResult::MISS;
    for (const auto& a : plates) {
        const double s = std::hypot(a.position[0], a.position[1]);
        double h, t;
        if (!ball_.simulate(pitch, s, h, t)) continue;
        const cv::Vec3d pb(s * std::cos(yaw), s * std::sin(yaw), h);
        if (!camera_.inFront(pb)) continue;
        const cv::Point2d px = camera_.project(pb);
        const auto poly = panelPolygon(a.det);
        const double inside = cv::pointPolygonTest(poly, cv::Point2f(px), true);
        // offset in panel coordinates, kept for the closest panel
        const cv::Point2d c = (cv::Point2d(poly[0]) + cv::Point2d(poly[1]) + cv::Point2d(poly[2]) + cv::Point2d(poly[3])) * 0.25;
        const double cd = cv::norm(px - c);
        if (cd < best_center_d) {
            best_center_d = cd;
            const cv::Point2d lm = (cv::Point2d(poly[0]) + cv::Point2d(poly[1])) * 0.5;
            const cv::Point2d rm = (cv::Point2d(poly[2]) + cv::Point2d(poly[3])) * 0.5;
            const cv::Point2d up = (cv::Point2d(poly[1]) - cv::Point2d(poly[0])) * 0.5;
            const cv::Point2d u = (rm - lm) * 0.5;
            const double nu = std::max(1e-6, u.dot(u)), nv = std::max(1e-6, up.dot(up));
            rec.off_u = (px - c).dot(u) / nu;
            rec.off_v = (px - c).dot(up) / nv;
            if (impact_px) *impact_px = px;
        }
        if (inside >= 0) result = ShotResult::HIT;
        best_miss = std::min(best_miss, std::max(0.0, -inside));
    }
    if (miss_px) *miss_px = best_miss;
    return result;
}

// ------------------------------------------------------------ evaluation

namespace {

struct Stats {
    std::vector<double> v;
    PredictionResult summarize(const std::string& name, double h) {
        PredictionResult r;
        r.name = name, r.horizon = h, r.samples = static_cast<int>(v.size());
        if (v.empty()) return r;
        std::sort(v.begin(), v.end());
        double s = 0;
        for (double x : v) s += x;
        r.mean_px = s / v.size();
        r.median_px = v[v.size() / 2];
        r.p90_px = v[std::min(v.size() - 1, static_cast<std::size_t>(0.9 * v.size()))];
        return r;
    }
};

double median(std::vector<double> v) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

void fireAndJudge(StrategyResult& sr, const JudgeFn& judge, const GimbalSim& g, const AimCommand& cmd,
                  const std::string& id, int frame, double latency, double stamp, ShotRecord* out) {
    ShotRecord rec;
    rec.frame = frame;
    judge(stamp + latency, g.yaw, g.pitch, cmd.t_fly, id, rec);
    ++sr.shots;
    if (rec.result != ShotResult::UNKNOWN) ++sr.verifiable;
    if (rec.result == ShotResult::HIT) ++sr.hits;
    sr.shots_log.push_back(rec);
    if (out) *out = rec;
}

}  // namespace

EvalResult evaluate(const Config& cfg, const std::vector<FrameObservations>& obs, const CameraModel& camera,
                    bool keep_log, const JudgeFn* custom_judge) {
    EvalResult res;
    const double fps = cfg.eval.fps;
    const double dt = 1.0 / fps;
    Ballistics ball(cfg.ballistic);
    RobotTracker robot(cfg.tracker, camera);
    PointTracker point(cfg.tracker, camera);
    RobotAimer robot_aimer(cfg.aimer, ball);
    PointAimer point_aimer(cfg.aimer, ball);
    DirectAimer direct_aimer(cfg.aimer, ball);
    GimbalSim g_robot(cfg.aimer.gimbal_tau, cfg.aimer.gimbal_max_rate, cfg.aimer.gimbal_ff);
    GimbalSim g_point = g_robot, g_direct = g_robot;
    ShotJudge video_judge(obs, camera, ball, fps);
    const JudgeFn default_judge = [&](double t, double yaw, double pitch, double tf, const std::string& id,
                                      ShotRecord& rec) {
        rec.result = video_judge.judge(t, yaw, pitch, tf, id, rec);
    };
    const JudgeFn& judge = custom_judge ? *custom_judge : default_judge;

    StrategyResult s_direct, s_point, s_robot, s_oracle;
    s_direct.name = "direct (aim at current plate)";
    s_point.name = "point KF (predict plate centre)";
    s_robot.name = "robot EKF (whole-robot model)";
    s_oracle.name = "oracle (cheats: aims at the future plate)";
    GimbalSim g_oracle = g_robot;
    double oracle_tf = 0.25;

    const std::size_t H = cfg.eval.horizons.size();
    std::vector<Stats> p_static(H), p_point(H), p_robot(H);
    std::vector<double> omegas, r1s, r2s, dzs;
    int tracking_frames = 0, frames_with_armor = 0;
    const double scale = cfg.aimer.fire_tolerance_scale;

    for (std::size_t n = 0; n < obs.size(); ++n) {
        const auto& f = obs[n];
        if (!f.armors.empty()) ++frames_with_armor;
        robot.update(f);
        point.update(f);

        // --- direct
        {
            const AimCommand c = direct_aimer.aim(f, f.stamp);
            if (c.valid) {
                g_direct.step(c.yaw, c.pitch, dt);
                const int idx = chooseInitialTarget(f.armors);
                if (c.fire && onTarget(g_direct, c, f.armors[idx].det.type, scale))
                    fireAndJudge(s_direct, judge, g_direct, c, deviceId(f.armors[idx].det), f.frame,
                                 cfg.aimer.latency, f.stamp, nullptr);
            }
        }
        // --- point KF
        {
            const AimCommand c = point_aimer.aim(point, f.stamp);
            if (c.valid) {
                g_point.step(c.yaw, c.pitch, dt);
                if (c.fire && onTarget(g_point, c, point.armor_type, scale))
                    fireAndJudge(s_point, judge, g_point, c, point.id(), f.frame, cfg.aimer.latency, f.stamp,
                                 nullptr);
            }
        }
        // --- robot EKF
        FrameLog log;
        log.frame = f.frame;
        log.stamp = f.stamp;
        log.matched = robot.matched_count;
        {
            const AimCommand c = robot_aimer.aim(robot, f.stamp);
            log.cmd = c;
            if (c.valid) {
                g_robot.step(c.yaw, c.pitch, dt);
                if (c.fire && onTarget(g_robot, c, robot.armor_type, scale)) {
                    fireAndJudge(s_robot, judge, g_robot, c, robot.id(), f.frame, cfg.aimer.latency, f.stamp,
                                 &log.shot_rec);
                    log.shot = true;
                }
            }
        }
        // --- oracle: looks up where the plate really is at impact time. Upper
        // bound of what this metric can award on this recording.
        {
            AimCommand c;
            std::vector<ArmorObservation> plates;
            for (int it = 0; it < 3; ++it) {
                if (!video_judge.platesAt(f.stamp + cfg.aimer.latency + oracle_tf, "", plates) || plates.empty())
                    break;
                const ArmorObservation* best = nullptr;
                double best_w = -1;
                for (const auto& p : plates) {
                    const double w = cv::norm(p.det.corners[3] - p.det.corners[0]);
                    if (w > best_w) best_w = w, best = &p;
                }
                const double s = std::hypot(best->position[0], best->position[1]);
                const BallisticSolution sol = ball.solve(s, best->position[2]);
                if (!sol.ok) break;
                c.valid = true;
                c.fire = true;
                c.yaw = std::atan2(best->position[1], best->position[0]);
                c.pitch = sol.pitch;
                c.t_fly = sol.t_fly;
                c.distance = cv::norm(best->position);
                c.aim_point = best->position;
                oracle_tf = sol.t_fly;
            }
            if (c.valid) {
                g_oracle.step(c.yaw, c.pitch, dt);
                if (onTarget(g_oracle, c, ArmorType::SMALL, scale))
                    fireAndJudge(s_oracle, judge, g_oracle, c, "", f.frame, cfg.aimer.latency, f.stamp, nullptr);
            }
        }

        log.state = robot.state();
        log.x = robot.x();
        log.gimbal_yaw = g_robot.yaw;
        log.gimbal_pitch = g_robot.pitch;
        if (keep_log) res.log.push_back(log);

        if (robot.state() == TrackState::TRACKING) {
            ++tracking_frames;
            const auto& x = robot.x();
            omegas.push_back(std::abs(x[RobotTracker::W]));
            r1s.push_back(x[RobotTracker::R1]);
            r2s.push_back(x[RobotTracker::R2]);
            dzs.push_back(x[RobotTracker::DZ]);
        }

        // --- prediction error, same sample set for all three methods
        const int static_idx = chooseInitialTarget(f.armors);
        if (robot.state() == TrackState::TRACKING && point.active() && static_idx >= 0) {
            for (std::size_t hi = 0; hi < H; ++hi) {
                const std::size_t m = n + static_cast<std::size_t>(std::lround(cfg.eval.horizons[hi] * fps));
                if (m >= obs.size()) continue;
                const auto& fm = obs[m];
                const RobotTracker::Vec xs = robot.predict(fm.stamp);
                const cv::Vec3d pp = point.position(point.predict(fm.stamp));
                const cv::Point2d pp_px = camera.project(pp);
                const cv::Point2d st_px(f.armors[static_idx].det.center);
                const double center_dir = std::atan2(xs[RobotTracker::YC], xs[RobotTracker::XC]);
                for (const auto& a : fm.armors) {
                    if (deviceId(a.det) != robot.id()) continue;
                    const cv::Point2d target(a.det.center);
                    double best = 1e18;
                    for (int k = 0; k < robot.armorsNum(); ++k) {
                        if (std::abs(wrapAngle(robot.armorYaw(xs, k) - center_dir)) > kPi / 2) continue;
                        best = std::min(best, cv::norm(camera.project(robot.armorPosition(xs, k)) - target));
                    }
                    if (best < 1e17) p_robot[hi].v.push_back(best);
                    p_point[hi].v.push_back(cv::norm(pp_px - target));
                    p_static[hi].v.push_back(cv::norm(st_px - target));
                }
            }
        }
    }

    for (auto* s : {&s_direct, &s_point, &s_robot, &s_oracle}) s->frames = static_cast<int>(obs.size());
    res.strategies = {s_direct, s_point, s_robot, s_oracle};
    for (std::size_t hi = 0; hi < H; ++hi) {
        const double h = cfg.eval.horizons[hi];
        res.predictions.push_back(p_static[hi].summarize("no prediction", h));
        res.predictions.push_back(p_point[hi].summarize("point KF", h));
        res.predictions.push_back(p_robot[hi].summarize("robot EKF", h));
    }
    res.tracking_ratio = obs.empty() ? 0 : double(tracking_frames) / obs.size();
    res.frames_with_armor_ratio = obs.empty() ? 0 : double(frames_with_armor) / obs.size();
    res.median_abs_omega = median(omegas);
    res.median_r1 = median(r1s);
    res.median_r2 = median(r2s);
    res.median_dz = median(dzs);
    return res;
}

std::string formatReport(const EvalResult& r, double fps) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(1);
    o << "| strategy | shots | verifiable | hits | hit rate | hits/s |\n|---|---|---|---|---|---|\n";
    for (const auto& s : r.strategies)
        o << "| " << s.name << " | " << s.shots << " | " << s.verifiable << " | " << s.hits << " | "
          << 100.0 * s.hit_rate() << "% | " << s.hits_per_second(fps) << " |\n";
    o << "\n| prediction | horizon | samples | mean px | median px | p90 px |\n|---|---|---|---|---|---|\n";
    for (const auto& p : r.predictions)
        o << "| " << p.name << " | " << std::setprecision(2) << p.horizon << " s | " << p.samples << " | "
          << std::setprecision(1) << p.mean_px << " | " << p.median_px << " | " << p.p90_px << " |\n";
    o << std::setprecision(3) << "\ntracking ratio " << r.tracking_ratio << ", frames with armor "
      << r.frames_with_armor_ratio << ", median |w| " << r.median_abs_omega << " rad/s, r1 " << r.median_r1
      << " m, r2 " << r.median_r2 << " m, dz " << r.median_dz << " m\n";
    return o.str();
}

}  // namespace autoaim
