// Minimal self-contained tests (no framework dependency).
#include <cmath>
#include <cstdio>
#include <random>

#include "autoaim/aimer.hpp"
#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
#include "autoaim/detector.hpp"
#include "autoaim/keypoint_detector.hpp"
#include "autoaim/pose.hpp"
#include "autoaim/pose_buffer.hpp"
#include "autoaim/tracker.hpp"

using namespace autoaim;

static int failures = 0;
#define CHECK(cond, ...)                                  \
    do {                                                  \
        if (!(cond)) {                                    \
            ++failures;                                   \
            std::printf("FAIL %s:%d  ", __FILE__, __LINE__); \
            std::printf(__VA_ARGS__);                     \
            std::printf("\n");                            \
        }                                                 \
    } while (0)

static void testBallistics() {
    BallisticConfig c;
    c.use_drag = false;
    Ballistics ideal(c);
    double h, t;
    CHECK(ideal.simulate(0.0, 5.0, h, t), "ideal simulate");
    const double t_exp = 5.0 / c.bullet_speed;
    CHECK(std::abs(t - t_exp) < 1e-9 && std::abs(h + 0.5 * c.gravity * t_exp * t_exp) < 1e-9,
          "ideal closed form h=%f t=%f", h, t);

    c.use_drag = true;
    Ballistics drag(c);
    double hd, td;
    CHECK(drag.simulate(0.0, 5.0, hd, td), "drag simulate");
    CHECK(td > t && hd < h, "drag must slow the bullet: t %f vs %f, h %f vs %f", td, t, hd, h);
    // RK4 with k = 0 must equal the closed form
    c.drag_k = 0.0;
    double h0, t0;
    Ballistics(c).simulate(0.1, 6.0, h0, t0);
    BallisticConfig ci = c;
    ci.use_drag = false;
    double h1, t1;
    Ballistics(ci).simulate(0.1, 6.0, h1, t1);
    CHECK(std::abs(h0 - h1) < 1e-6 && std::abs(t0 - t1) < 1e-6, "rk4 vs closed form %g %g", h0 - h1, t0 - t1);

    for (double s : {2.0, 5.0, 8.0})
        for (double z : {-0.3, 0.0, 0.4}) {
            const auto sol = drag.solve(s, z);
            CHECK(sol.ok, "solve %f %f", s, z);
            double hh, tt;
            drag.simulate(sol.pitch, s, hh, tt);
            CHECK(std::abs(hh - z) < 1e-3, "solve accuracy s=%f z=%f got %f", s, z, hh);
        }

    // Golden values captured from the implementation before the bullet-speed
    // rework (default 17 mm config).  The tolerance was 1e-9 until the
    // warm-start optimisation of solve() (vacuum-drop initial guess, sqrt in
    // the RK4 derivative): it converges to the same solution within the
    // solver's own |dh| < 5e-4 m criterion, with a measured pitch deviation
    // of at most 6.4e-5 rad (~0.004 deg, vs the 0.2 deg minimum fire
    // tolerance) for a 2.7x speedup.  2e-4 rad still catches real drift.
    {
        Ballistics golden{ BallisticConfig{} };
        const auto s1 = golden.solve(5.0, 0.5);
        CHECK(s1.ok && std::abs(s1.pitch - 0.141715304681) < 2e-4 &&
                  std::abs(s1.t_fly - 0.212001575796) < 1e-5,
              "golden solve(5, 0.5): pitch %.12g t %.12g", s1.pitch, s1.t_fly);
        const auto s2 = golden.solve(8.0, 0.5);
        CHECK(s2.ok && std::abs(s2.pitch - 0.132565507948) < 2e-4 &&
                  std::abs(s2.t_fly - 0.348777463964) < 1e-5,
              "golden solve(8, 0.5): pitch %.12g t %.12g", s2.pitch, s2.t_fly);
    }
    // Measured-speed override: a slower bullet needs more pitch and flies
    // longer; clearing the override restores the config solution exactly.
    {
        Ballistics b{ BallisticConfig{} };
        b.setMeasuredBulletSpeed(23.0);
        CHECK(b.hasMeasuredSpeed() && std::abs(b.bulletSpeed() - 23.0) < 1e-12, "override active");
        const auto slow = b.solve(5.0, 0.5);
        CHECK(slow.ok && slow.pitch > 0.141715304681 && slow.t_fly > 0.212001575796,
              "slower bullet: pitch %f t %f", slow.pitch, slow.t_fly);
        b.setMeasuredBulletSpeed(0.0);
        CHECK(!b.hasMeasuredSpeed(), "override cleared");
        const auto back = b.solve(5.0, 0.5);
        CHECK(std::abs(back.pitch - 0.141715304681) < 2e-4, "golden restored after clear");
    }
    // 42 mm class (hero): its own speed/drag defaults, solvable, and slower
    // than 17 mm over the same shot.
    {
        BallisticConfig c42;
        c42.bullet_mm = 42;
        Ballistics hero(c42);
        CHECK(hero.bulletMm() == 42 && std::abs(hero.bulletSpeed() - c42.bullet_speed_42) < 1e-12 &&
                  std::abs(hero.dragK() - c42.drag_k_42) < 1e-12,
              "42 mm class params");
        const auto s42 = hero.solve(8.0, 0.5);
        const auto s17 = Ballistics{ BallisticConfig{} }.solve(8.0, 0.5);
        CHECK(s42.ok && s17.ok && s42.t_fly > s17.t_fly, "42 mm flies longer: %f vs %f", s42.t_fly,
              s17.t_fly);
        hero.setBulletClass(17);
        CHECK(hero.bulletMm() == 17 && std::abs(hero.bulletSpeed() - 25.0) < 1e-12, "class switch back");
        hero.setBulletClass(20);  // unknown class falls back to 17
        CHECK(hero.bulletMm() == 17, "unknown class -> 17");
    }
}

static void testPose() {
    CameraConfig cc;
    cc.pitch_deg = 3.0;
    CameraModel cam(cc);
    PoseConfig pc;
    PoseSolver solver(cam, pc);
    std::mt19937 rng(1);
    std::normal_distribution<double> n(0, 0.3);
    double max_pos = 0, max_yaw = 0;
    for (double dist : {2.0, 4.0, 6.0})
        for (double yaw_off : {-40.0, -15.0, 10.0, 35.0}) {
            const cv::Vec3d p(dist, 0.3, -0.15);
            const double yaw = std::atan2(p[1], p[0]) + yaw_off * kPi / 180;
            const auto c3 = armorCorners(p, yaw, 15 * kPi / 180, ArmorType::SMALL);
            ArmorDetection d;
            d.type = ArmorType::SMALL;
            for (int i = 0; i < 4; ++i) {
                const cv::Point2d q = cam.project(c3[i]);
                d.corners[i] = cv::Point2f(float(q.x + n(rng)), float(q.y + n(rng)));
            }
            d.center = (d.corners[0] + d.corners[1] + d.corners[2] + d.corners[3]) * 0.25f;
            ArmorObservation o;
            CHECK(solver.solve(d, o), "pose solve");
            max_pos = std::max(max_pos, cv::norm(o.position - p) / dist);
            max_yaw = std::max(max_yaw, std::abs(wrapAngle(o.yaw - yaw)) * 180 / kPi);
        }
    CHECK(max_pos < 0.03, "relative position error %.3f", max_pos);
    CHECK(max_yaw < 8.0, "yaw error %.1f deg", max_yaw);
    std::printf("pose: max rel. position error %.2f%%, max yaw error %.2f deg (0.3 px noise)\n", 100 * max_pos,
                max_yaw);
}

static void testTracker() {
    CameraConfig cc;
    CameraModel cam(cc);
    TrackerConfig tc;
    RobotTracker tr(tc, cam);
    const double w = 3.0, r = 0.24, fps = 50;
    const cv::Vec3d c(5, 0.2, -0.1);
    for (int n = 0; n < 200; ++n) {
        const double t = n / fps;
        FrameObservations f;
        f.frame = n, f.stamp = t;
        const double los = std::atan2(c[1], c[0]);
        for (int k = 0; k < 4; ++k) {
            const double yaw = 0.2 + w * t + kPi / 2 * k;
            if (std::abs(wrapAngle(yaw - los)) > 1.0) continue;
            ArmorObservation o;
            o.position = {c[0] - r * std::cos(yaw), c[1] - r * std::sin(yaw), c[2]};
            o.distance = cv::norm(o.position);
            o.yaw = wrapAngle(yaw);
            o.det.number = "3";
            o.det.left.length = o.det.right.length = 20;
            o.image_center_dist = 100;
            f.armors.push_back(o);
        }
        tr.update(f);
    }
    const auto& x = tr.x();
    CHECK(tr.state() == TrackState::TRACKING, "tracking state %s", toString(tr.state()));
    CHECK(std::abs(x[RobotTracker::W] - w) < 0.2, "omega %f", x[RobotTracker::W]);
    CHECK(std::hypot(x[RobotTracker::XC] - c[0], x[RobotTracker::YC] - c[1]) < 0.05, "centre %f %f",
          x[RobotTracker::XC], x[RobotTracker::YC]);
    CHECK(std::abs(x[RobotTracker::R1] - r) < 0.03, "radius %f", x[RobotTracker::R1]);
    std::printf("tracker: w %.3f (true %.1f), r1 %.3f r2 %.3f (true %.2f)\n", x[RobotTracker::W], w,
                x[RobotTracker::R1], x[RobotTracker::R2], r);
}

static void testPoseBuffer() {
    // Gimbal spinning at 3 rad/s (the G-02 reference case), pose samples at
    // 200 Hz like a serial IMU stream, frames queried at in-between times.
    const double w = 3.0;
    PoseBuffer buf(512, 0.05);
    for (int n = 0; n <= 440; ++n) {  // cover the last queried frame stamp (2.063 s)
        const double t = n * 0.005;
        buf.push({t, quatFromYawPitchRoll(w * t, 0.0, 0.0)});
    }
    double max_aligned = 0, max_latest = 0;
    for (int n = 0; n < 50; ++n) {
        const double t = 0.1 + n * 0.04 + 0.003;  // 25 fps frames, off-sample phase
        Quat q;
        CHECK(buf.query(t, q), "pose query at %f", t);
        max_aligned = std::max(max_aligned, quatAngleBetween(q, quatFromYawPitchRoll(w * t, 0, 0)));
        // The pre-G-02 behaviour: newest sample not newer than the frame.
        const double t_latest = std::floor(t / 0.005) * 0.005;
        max_latest = std::max(max_latest, quatAngleBetween(quatFromYawPitchRoll(w * t_latest, 0, 0),
                                                           quatFromYawPitchRoll(w * t, 0, 0)));
    }
    // SLERP between exact samples of a constant-rate rotation is exact; the
    // G-02 alignment budget is 0.3 ms == 0.0009 rad at 3 rad/s.
    CHECK(max_aligned < 9e-4, "aligned error %g rad exceeds the G-02 budget", max_aligned);
    CHECK(max_latest > 10 * max_aligned, "latest-sample error should dwarf aligned: %g vs %g", max_latest,
          max_aligned);
    std::printf("pose buffer: aligned err %.2e rad, latest-sample err %.2e rad at 3 rad/s\n", max_aligned,
                max_latest);

    Quat q;
    CHECK(!buf.query(-0.1, q), "query before coverage must fail");
    CHECK(!buf.query(2.3, q), "query after coverage must fail");

    PoseBuffer gappy(16, 0.02);
    gappy.push({0.0, quatFromYawPitchRoll(0, 0, 0)});
    gappy.push({0.1, quatFromYawPitchRoll(0.3, 0, 0)});
    double gap = 0;
    CHECK(!gappy.query(0.05, q, gap), "query across a dropped-frame gap must fail");
    CHECK(std::abs(gap - 0.1) < 1e-12, "reported gap %f", gap);

    PoseBuffer ooo(16, 0.05);  // out-of-order arrival still queries correctly
    ooo.push({0.010, quatFromYawPitchRoll(0.03, 0, 0)});
    ooo.push({0.000, quatFromYawPitchRoll(0.00, 0, 0)});
    ooo.push({0.005, quatFromYawPitchRoll(0.015, 0, 0)});
    CHECK(ooo.query(0.0075, q), "out-of-order query");
    CHECK(quatAngleBetween(q, quatFromYawPitchRoll(0.0225, 0, 0)) < 1e-9, "out-of-order slerp");
}

static void testClockAligner() {
    ClockAligner al(64);
    std::mt19937 rng(2);
    std::normal_distribution<double> jitter(0, 0.001);
    const double true_offset = 12.345;
    for (int n = 0; n < 64; ++n) {
        const double remote = n * 0.005;
        double local = remote + true_offset + jitter(rng);
        if (n % 17 == 5) local += 0.05;  // transfer outlier the median must reject
        al.addPair(remote, local);
    }
    CHECK(al.ready(), "aligner ready");
    CHECK(std::abs(al.offset() - true_offset) < 0.002, "offset %f vs %f", al.offset(), true_offset);
    CHECK(std::abs(al.toLocal(100.0) - (100.0 + true_offset)) < 0.002, "toLocal");
    std::printf("clock aligner: offset error %.2e s\n", std::abs(al.offset() - true_offset));
}

static void testDeviceId() {
    ArmorDetection d;
    d.number = "3";
    CHECK(deviceId(d) == "R3:small", "base id %s", deviceId(d).c_str());
    ArmorDetection blue = d;
    blue.color = Color::BLUE;
    CHECK(deviceId(blue) == "B3:small", "blue id %s", deviceId(blue).c_str());
    ArmorDetection large = d;
    large.type = ArmorType::LARGE;
    CHECK(deviceId(large) == "R3:large", "large id %s", deviceId(large).c_str());
    ArmorDetection other = d;
    other.number = "4";
    CHECK(deviceId(other) != deviceId(d), "number matters");

    // A tracker must not fuse a same-number plate of another colour or size:
    // both teams field a "3", and the wrong fusion would teleport the track.
    CameraConfig cc;
    CameraModel cam(cc);
    TrackerConfig tc;
    PointTracker pt(tc, cam);
    auto mk = [](Color col, ArmorType ty, const cv::Vec3d& p, double stamp) {
        FrameObservations f;
        f.stamp = stamp;
        ArmorObservation o;
        o.position = p;
        o.distance = cv::norm(p);
        o.yaw = 0;
        o.det.number = "3";
        o.det.color = col;
        o.det.type = ty;
        f.armors.push_back(o);
        return f;
    };
    pt.update(mk(Color::RED, ArmorType::SMALL, {4, 0, 0}, 0.0));
    CHECK(pt.active() && pt.id() == "R3:small", "init id %s", pt.id().c_str());
    pt.update(mk(Color::BLUE, ArmorType::SMALL, {-4, 3, 0}, 0.1));  // foreign "3"
    CHECK(pt.id() == "R3:small", "id after foreign obs %s", pt.id().c_str());
    const cv::Vec3d predicted = pt.position(pt.predict(0.2));
    pt.update(mk(Color::RED, ArmorType::LARGE, {-4, -3, 0}, 0.2));  // right colour, wrong size
    CHECK(pt.id() == "R3:small", "id after large obs %s", pt.id().c_str());
    CHECK(cv::norm(pt.position(pt.predict(0.2)) - predicted) < 1e-6, "a foreign plate was fused");
    std::printf("device id: same-number foreign plates are not fused\n");
}

static void testAimerRates() {
    // B-01: a target circling at a known rate must produce aim commands whose
    // yaw_rate matches it; pitch_rate stays ~0 for level motion.
    CameraConfig cc;
    CameraModel cam(cc);
    TrackerConfig tc;
    PointTracker pt(tc, cam);
    Ballistics ball{ BallisticConfig{} };
    AimerConfig ac;
    PointAimer aimer(ac, ball);
    const double w = 0.6, radius = 3.0, fps = 50;
    double sum_rate = 0, sum_pitch = 0;
    int samples = 0;
    double max_abs_rate = 0;
    for (int n = 0; n < 200; ++n) {
        const double t = n / fps;
        FrameObservations f;
        f.frame = n, f.stamp = t;
        ArmorObservation o;
        o.position = {radius * std::cos(w * t), radius * std::sin(w * t), 0.0};
        o.distance = cv::norm(o.position);
        o.yaw = w * t;
        o.det.number = "3";
        f.armors.push_back(o);
        pt.update(f);
        const auto cmd = aimer.aim(pt, t);
        CHECK(cmd.valid, "aim valid at %d", n);
        max_abs_rate = std::max(max_abs_rate, std::abs(cmd.yaw_rate));
        if (t > 2.0) {  // KF settled, rate state warm
            sum_rate += cmd.yaw_rate;
            sum_pitch += cmd.pitch_rate;
            ++samples;
        }
    }
    CHECK(samples > 50, "samples %d", samples);
    CHECK(std::abs(sum_rate / samples - w) < 0.1, "yaw_rate %.3f vs %.1f", sum_rate / samples, w);
    CHECK(std::abs(sum_pitch / samples) < 0.05, "pitch_rate %.3f", sum_pitch / samples);
    CHECK(max_abs_rate <= ac.gimbal_max_rate + 1e-9, "rate clamp %.3f", max_abs_rate);
    std::printf("aimer rates: yaw_rate %.3f (true %.1f), pitch_rate %.3f\n", sum_rate / samples, w,
                sum_pitch / samples);
}

static void testKeypointDetector() {
    // Synthetic YOLO-pose output: 2 classes -> 18 channels, letterboxed
    // 1440x1080 into 640 (scale 4/9, pad_y 80).  No trained model involved.
    const int C = 18, N = 4;
    const int sizes[3] = {1, C, N};
    cv::Mat out(3, sizes, CV_32F, cv::Scalar(0));
    auto set = [&](int c, int i, float v) { out.ptr<float>()[c * N + i] = v; };
    auto anchor = [&](int i, float cx, float cy, float w, float h, int cls, float score,
                      const std::array<cv::Point2f, 4>& kp) {
        set(0, i, cx); set(1, i, cy); set(2, i, w); set(3, i, h);
        set(4 + cls, i, score);
        for (int k = 0; k < 4; ++k) {
            set(6 + 3 * k, i, kp[k].x);
            set(6 + 3 * k + 1, i, kp[k].y);
            set(6 + 3 * k + 2, i, 0.99f);
        }
    };
    const std::array<cv::Point2f, 4> kp0 = {cv::Point2f(270, 300), cv::Point2f(370, 300),
                                            cv::Point2f(375, 345), cv::Point2f(265, 345)};
    anchor(0, 320, 322, 120, 50, 0, 0.9f, kp0);
    anchor(1, 324, 324, 120, 50, 0, 0.8f, kp0);  // duplicate -> NMS drops it
    anchor(2, 100, 100, 60, 30, 1, 0.2f, kp0);   // below conf -> dropped
    const std::array<cv::Point2f, 4> kp3 = {cv::Point2f(65, 545), cv::Point2f(135, 545),
                                            cv::Point2f(138, 578), cv::Point2f(62, 578)};
    anchor(3, 100, 560, 80, 40, 1, 0.7f, kp3);

    const auto cands = parseKeypointOutput(out, 2, 0.25f, 0.45f, 640, 1440, 1080);
    CHECK(cands.size() == 2, "candidates %zu", cands.size());
    CHECK(std::abs(cands[0].score - 0.9f) < 1e-6 && cands[0].cls == 0, "best candidate first");
    CHECK(cands[1].cls == 1 && std::abs(cands[1].score - 0.7f) < 1e-6, "second candidate");

    DetectorConfig dc;
    dc.onnx_classes = {"3", "1"};
    const auto armor = candidateToArmor(cands[0], dc);
    CHECK(armor.number == "3", "number %s", armor.number.c_str());
    CHECK(armor.type == ArmorType::SMALL, "small plate by aspect");
    const std::array<cv::Point2f, 4> want = {cv::Point2f(596.25f, 596.25f), cv::Point2f(607.5f, 495.f),
                                             cv::Point2f(832.5f, 495.f), cv::Point2f(843.75f, 596.25f)};
    for (int i = 0; i < 4; ++i)
        CHECK(cv::norm(armor.corners[i] - want[i]) < 0.01, "corner %d (%.2f, %.2f)", i,
              armor.corners[i].x, armor.corners[i].y);
    CHECK(armor.color == dc.enemy_color, "colour filled");

    // The transposed export layout ([1, N, C]) parses identically.
    const int sizes_t[3] = {1, N, C};
    cv::Mat out_t(3, sizes_t, CV_32F, cv::Scalar(0));
    for (int c = 0; c < C; ++c)
        for (int i = 0; i < N; ++i) out_t.ptr<float>()[i * C + c] = out.ptr<float>()[c * N + i];
    CHECK(parseKeypointOutput(out_t, 2, 0.25f, 0.45f, 640, 1440, 1080).size() == 2,
          "transposed layout");

    // Detector fallback: backend=onnx with no model must run traditional.
    DetectorConfig fc;
    fc.backend = "onnx";
    fc.onnx_path = "";
    Detector det(fc);
    CHECK(!det.usingOnnx(), "fallback when no model");
    const cv::Mat black(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    CHECK(det.detect(black).empty(), "traditional path on black frame");
    KeypointDetector missing(fc);
    CHECK(!missing.ok() && missing.detect(black).empty(), "missing model is inert");
    std::printf("keypoint detector: synthetic-tensor postprocess + fallback ok\n");
}

int main() {
    testBallistics();
    testPose();
    testTracker();
    testDeviceId();
    testAimerRates();
    testKeypointDetector();
    testPoseBuffer();
    testClockAligner();
    if (failures == 0) std::printf("ALL TESTS PASSED\n");
    return failures == 0 ? 0 : 1;
}
