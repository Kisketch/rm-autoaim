#include "autoaim/pose_buffer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoaim {

namespace {

Quat quatMul(const Quat& a, const Quat& b) {
    // Hamilton product, (w, x, y, z).
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

Quat quatAxisAngle(double ax, double ay, double az, double angle) {
    const double s = std::sin(angle / 2);
    return {std::cos(angle / 2), ax * s, ay * s, az * s};
}

}  // namespace

Quat normalizeQuat(Quat q) {
    const double n = std::sqrt(q.dot(q));
    if (n < 1e-12) return {1.0, 0.0, 0.0, 0.0};
    return q / n;
}

Quat slerpQuat(const Quat& a, const Quat& b, double t) {
    Quat bb = b;
    double dot = a.dot(b);
    if (dot < 0) {  // same rotation, opposite hemisphere: take the short arc
        bb = -b;
        dot = -dot;
    }
    if (dot > 0.9995) {  // tiny angle: linear blend is indistinguishable
        return normalizeQuat(a + t * (bb - a));
    }
    dot = std::min(1.0, dot);
    const double theta = std::acos(dot);
    const double s = std::sin(theta);
    const double wa = std::sin((1 - t) * theta) / s;
    const double wb = std::sin(t * theta) / s;
    return wa * a + wb * bb;
}

double quatAngleBetween(const Quat& a, const Quat& b) {
    const double dot = std::min(1.0, std::abs(normalizeQuat(a).dot(normalizeQuat(b))));
    return 2 * std::acos(dot);
}

Quat quatFromYawPitchRoll(double yaw, double pitch, double roll) {
    const Quat qz = quatAxisAngle(0, 0, 1, yaw);
    const Quat qy = quatAxisAngle(0, 1, 0, pitch);
    const Quat qx = quatAxisAngle(1, 0, 0, roll);
    return quatMul(qz, quatMul(qy, qx));
}

cv::Matx33d quatToMat(const Quat& q) {
    const Quat u = normalizeQuat(q);
    const double w = u[0], x = u[1], y = u[2], z = u[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
            2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
            2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
}

PoseBuffer::PoseBuffer(std::size_t capacity, double max_gap, double max_extrapolation)
    : capacity_(capacity == 0 ? 1 : capacity), max_gap_(max_gap),
      max_extrap_(std::max(0.0, max_extrapolation)) {}

void PoseBuffer::push(GimbalPoseSample s) {
    // A non-finite sample (e.g. a corrupt serial frame decoding to NaN
    // floats) must never enter the buffer: NaN comparisons would corrupt
    // the time ordering and poison every later interpolation.  Dropping it
    // makes the affected frames miss their query and get skipped, which is
    // the designed degradation.
    if (!std::isfinite(s.stamp) || !std::isfinite(s.quat[0]) || !std::isfinite(s.quat[1]) ||
        !std::isfinite(s.quat[2]) || !std::isfinite(s.quat[3]))
        return;
    s.quat = normalizeQuat(s.quat);
    std::lock_guard<std::mutex> lock(mu_);
    // Serial samples normally arrive in order; only walk back over the few
    // that a jittered arrival could have overtaken.
    auto it = buf_.end();
    while (it != buf_.begin() && (it - 1)->stamp > s.stamp) --it;
    if (it != buf_.begin() && (it - 1)->stamp == s.stamp) {
        (it - 1)->quat = s.quat;  // re-stamped sample replaces, never duplicates
        return;
    }
    buf_.insert(it, s);
    while (buf_.size() > capacity_) buf_.pop_front();
}

bool PoseBuffer::query(double stamp, Quat& out) const {
    double gap = 0;
    return query(stamp, out, gap);
}

bool PoseBuffer::query(double stamp, Quat& out, double& gap) const {
    std::lock_guard<std::mutex> lock(mu_);
    if (buf_.empty() || !std::isfinite(stamp) || stamp < buf_.front().stamp) return false;
    if (stamp > buf_.back().stamp) {
        // Short extrapolation past the newest sample (SLERP with t > 1 follows
        // the same great circle at the same angular rate).
        if (buf_.size() < 2 || stamp - buf_.back().stamp > max_extrap_) return false;
        const GimbalPoseSample& s1 = buf_.back();
        const GimbalPoseSample& s0 = buf_[buf_.size() - 2];
        gap = s1.stamp - s0.stamp;
        if (gap <= 0 || gap > max_gap_) return false;
        out = normalizeQuat(slerpQuat(s0.quat, s1.quat, (stamp - s0.stamp) / gap));
        return true;
    }
    // First sample with stamp >= query stamp.
    std::size_t lo = 0, hi = buf_.size();
    while (lo < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (buf_[mid].stamp < stamp) lo = mid + 1; else hi = mid;
    }
    if (lo < buf_.size() && buf_[lo].stamp == stamp) {
        out = buf_[lo].quat;
        gap = 0;
        return true;
    }
    // lo >= 1 here: stamp > front was checked above and no exact match exists.
    const GimbalPoseSample& s1 = buf_[lo];
    const GimbalPoseSample& s0 = buf_[lo - 1];
    gap = s1.stamp - s0.stamp;
    if (gap > max_gap_) return false;
    out = slerpQuat(s0.quat, s1.quat, (stamp - s0.stamp) / gap);
    return true;
}

std::size_t PoseBuffer::size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return buf_.size();
}

bool PoseBuffer::empty() const {
    std::lock_guard<std::mutex> lock(mu_);
    return buf_.empty();
}

double PoseBuffer::oldestStamp() const {
    std::lock_guard<std::mutex> lock(mu_);
    return buf_.empty() ? 0.0 : buf_.front().stamp;
}

double PoseBuffer::newestStamp() const {
    std::lock_guard<std::mutex> lock(mu_);
    return buf_.empty() ? 0.0 : buf_.back().stamp;
}

ClockAligner::ClockAligner(std::size_t window) : window_(window == 0 ? 1 : window) {}

void ClockAligner::addPair(double remote_stamp, double local_stamp) {
    diffs_.push_back(local_stamp - remote_stamp);
    while (diffs_.size() > window_) diffs_.pop_front();
}

bool ClockAligner::ready() const { return !diffs_.empty(); }

double ClockAligner::offset() const {
    if (diffs_.empty()) return 0.0;
    std::vector<double> v(diffs_.begin(), diffs_.end());
    const std::size_t mid = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + mid, v.end());
    return v[mid];
}

void ClockAligner::reset() { diffs_.clear(); }

}  // namespace autoaim
