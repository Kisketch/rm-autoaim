// Minimal self-contained tests (no framework dependency).
#include <cmath>
#include <cstdio>
#include <random>

#include "autoaim/ballistics.hpp"
#include "autoaim/config.hpp"
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

int main() {
    testBallistics();
    testPose();
    testTracker();
    testPoseBuffer();
    testClockAligner();
    if (failures == 0) std::printf("ALL TESTS PASSED\n");
    return failures == 0 ? 0 : 1;
}
