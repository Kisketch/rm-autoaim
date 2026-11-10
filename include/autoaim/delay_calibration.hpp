// Camera -> attitude time-offset calibration from a gimbal sweep.
//
// Why: the attitude used for a frame is looked up at the frame's stamp.  If
// that stamp is late by D_cam (exposure + readout + transfer, folded in by
// "stamp = receipt time" or by a clock aligner) while attitude samples are late
// by a different D_att, every frame is rotated into the world with the attitude
// from (D_cam - D_att) later.  While the gimbal turns at w that is an angular
// error of w * (D_cam - D_att) applied to the MEASUREMENT - e.g. 10 ms during a
// 6 rad/s plate switch = 3.4 deg, ~30 cm at 5 m.  aimer.latency cannot absorb
// it: it is not a prediction horizon, it is a wrong rotation, and it is
// largest exactly when the gimbal slews.  ClockAligner (pose_buffer.hpp) folds
// the camera's median transfer delay into its offset, so camera stamps are
// late by exactly such a D_cam.
//
// How: point at a STATIC plate and sweep the gimbal.  For each candidate
// offset d the plate's world position is R_wg(stamp - d) * p_gimbal; the right
// d makes those positions stand still.  The estimate is the d that minimises
// their spread; feed it back as the camera stamp offset (rm-autoaim:
// MvsConfig::stamp_offset_us; rm-sentry-integration: sentry_robot
// --exposure-offset-ms), added to whatever offset the recording already used.
#pragma once

#include <opencv2/core.hpp>

#include <functional>
#include <vector>

namespace autoaim {

struct DelayEstimate {
    bool ok = false;
    double delay = 0.0;          // s: subtract from the frame stamp used while recording
    double spread = 0.0;         // RMS world-position scatter at `delay` (m)
    double spread_at_zero = 0.0; // the same at d = 0 (m)
    double gimbal_rate_rms = 0.0;  // rad/s, excitation during the recording
    int frames = 0;              // frames used
    const char* reason = "";     // why !ok
};

class DelayCalibrator {
public:
    // Candidate offsets [d_min, d_max] (seconds) on a grid of `step`.
    explicit DelayCalibrator(double d_min = -0.01, double d_max = 0.05, double step = 0.0005);

    // Attitude lookup: R_world_gimbal at host time t; false if not available.
    using AttitudeFn = std::function<bool(double t, cv::Matx33d& R_wg)>;

    // Queue one frame: its stamp and the plate position in the GIMBAL frame
    // (R_wg_used^T * world position).  Resolved later, once attitude samples up
    // to stamp - d_min have arrived.
    void add(double stamp, const cv::Vec3d& p_gimbal);
    // Look up the attitude grid for every queued frame with stamp - d_min <=
    // t_ready.  Frames missing any grid sample are dropped (keeps every
    // candidate d evaluated on the same frame set).
    void resolve(double t_ready, const AttitudeFn& attitude);
    std::size_t resolvedFrames() const { return frames_.size(); }

    DelayEstimate estimate() const;

private:
    struct Frame {
        double stamp;
        std::vector<cv::Vec3d> world;  // one per grid offset
    };
    double d_min_, step_;
    int n_;
    std::vector<std::pair<double, cv::Vec3d>> pending_;
    std::vector<Frame> frames_;
    std::vector<double> rate_sq_;  // gimbal angular rate^2 samples
};

}  // namespace autoaim
