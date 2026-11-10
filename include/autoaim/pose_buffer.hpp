// Timestamped gimbal-pose ring buffer with quaternion interpolation.
//
// Implements step 2 of docs/timestamp_design.md (G-02): the serial link delivers
// IMU quaternions with timestamps; the pose that belongs to a frame is the one
// interpolated at the frame's *exposure* timestamp, not the newest sample.
// Design budget from that document: image->pose alignment error <= 0.3 ms,
// which at a 3 rad/s spin is ~0.05 deg - interpolation between bracketing
// samples meets it easily, "take the latest sample" does not (up to one full
// serial period of error, e.g. 10 ms at 100 Hz = 1.7 deg).
//
// Frames: world is gimbal-centred, x forward, y left, z up (see types.hpp).
// Quaternions are (w, x, y, z), unit length, rotating world -> gimbal or the
// inverse - the buffer is convention-agnostic, but every sample pushed into
// one buffer must use the same convention.
#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <deque>
#include <mutex>

namespace autoaim {

// Unit quaternion, (w, x, y, z).
using Quat = cv::Vec4d;

Quat normalizeQuat(Quat q);
// Spherical linear interpolation, t in [0, 1]. Handles the double cover
// (q and -q are the same rotation) by flipping b onto a's hemisphere.
Quat slerpQuat(const Quat& a, const Quat& b, double t);
// Rotation angle between two orientations, radians in [0, pi].
double quatAngleBetween(const Quat& a, const Quat& b);
// ZYX Euler -> quaternion: yaw about z, then pitch about y, then roll about x.
// Matches the world frame above (x forward, y left, z up) and the yaw/pitch/
// roll triples the serial protocol already carries.
Quat quatFromYawPitchRoll(double yaw, double pitch, double roll);
cv::Matx33d quatToMat(const Quat& q);

struct GimbalPoseSample {
    double stamp = 0.0;  // seconds, host clock (see ClockAligner)
    Quat quat{1.0, 0.0, 0.0, 0.0};
};

// Fixed-capacity, time-ordered buffer of gimbal pose samples.
// Thread-safe: the serial thread pushes, the vision thread queries.
class PoseBuffer {
public:
    // capacity: samples kept (at 200 Hz, 512 samples cover ~2.5 s).
    // max_gap:  bracketing samples farther apart than this are NOT
    //           interpolated across - a gap that large means dropped serial
    //           frames, and a pose guessed across it can be worse than none.
    // max_extrapolation: how far past the NEWEST sample a query may go
    //           (constant angular velocity from the last two samples).  The
    //           attitude stream always trails the camera a little - a frame
    //           stamped at receipt is newer than the last attitude sample
    //           that has arrived - so a strict "must be bracketed" rule
    //           rejects every live frame.  0 keeps the strict behaviour.
    explicit PoseBuffer(std::size_t capacity = 512, double max_gap = 0.05,
                        double max_extrapolation = 0.0);

    // Insert a sample. Out-of-order arrivals (serial jitter) are placed at
    // their sorted position; a sample with an existing stamp replaces it.
    void push(GimbalPoseSample s);

    // Interpolated pose at `stamp`. Returns false when the stamp is outside
    // the buffered range (beyond the newest sample by more than
    // max_extrapolation) or the bracketing gap exceeds max_gap; the caller
    // decides the fallback (skip the frame, or hold fire) - silently using a
    // stale pose is exactly the failure mode this class exists to remove.
    bool query(double stamp, Quat& out) const;
    // As above, and reports the bracketing gap (s) for logging/acceptance.
    bool query(double stamp, Quat& out, double& gap) const;

    std::size_t size() const;
    bool empty() const;
    double oldestStamp() const;  // 0 when empty
    double newestStamp() const;  // 0 when empty

private:
    std::deque<GimbalPoseSample> buf_;
    std::size_t capacity_;
    double max_gap_;
    double max_extrap_;
    mutable std::mutex mu_;
};

// Maps a remote clock (MCU time, camera device time) onto the host clock.
// The offset is the median of the most recent (local - remote) pair
// differences, which rejects the occasional long transfer. Any *constant*
// transfer delay folds into the offset.  That part is harmless for the
// prediction horizon (aimer.latency is measured end to end), but NOT for the
// attitude lookup: a camera stamp that is late by D relative to the attitude
// stream rotates every measurement by gimbal_rate x D (largest during slews
// and plate switches).  Remove the constant with a measured offset - see
// delay_calibration.hpp (rm-autoaim: MvsConfig::stamp_offset_us;
// rm-sentry-integration: sentry_robot --calib-delay).  What the median
// itself removes is jitter.
class ClockAligner {
public:
    explicit ClockAligner(std::size_t window = 64);

    void addPair(double remote_stamp, double local_stamp);
    bool ready() const;
    double offset() const;  // local - remote; 0 when not ready
    double toLocal(double remote_stamp) const { return remote_stamp + offset(); }
    void reset();

private:
    std::deque<double> diffs_;
    std::size_t window_;
};

}  // namespace autoaim
