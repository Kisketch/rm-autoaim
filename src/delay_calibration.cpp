#include "autoaim/delay_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace autoaim {

DelayCalibrator::DelayCalibrator(double d_min, double d_max, double step)
    : d_min_(d_min), step_(step > 0 ? step : 0.0005),
      n_(std::max(3, static_cast<int>(std::floor((d_max - d_min) / (step > 0 ? step : 0.0005) + 1e-9)) + 1)) {}

void DelayCalibrator::add(double stamp, const cv::Vec3d& p_gimbal) {
    if (std::isfinite(stamp) && std::isfinite(p_gimbal[0]) && std::isfinite(p_gimbal[1]) &&
        std::isfinite(p_gimbal[2]))
        pending_.emplace_back(stamp, p_gimbal);
}

void DelayCalibrator::resolve(double t_ready, const AttitudeFn& attitude) {
    std::vector<std::pair<double, cv::Vec3d>> keep;
    for (const auto& [stamp, pg] : pending_) {
        if (stamp - d_min_ > t_ready) {  // newest needed attitude not here yet
            keep.emplace_back(stamp, pg);
            continue;
        }
        Frame f;
        f.stamp = stamp;
        f.world.reserve(n_);
        bool complete = true;
        cv::Matx33d R, R0;
        for (int i = 0; i < n_ && complete; ++i) {
            complete = attitude(stamp - (d_min_ + i * step_), R);
            if (complete) f.world.push_back(R * pg);
            if (i == 0) R0 = R;
        }
        if (!complete) continue;
        // excitation: angular rate over the grid span (rotation angle / time)
        const cv::Matx33d dR = R0.t() * R;
        const double c = std::clamp((dR(0, 0) + dR(1, 1) + dR(2, 2) - 1) * 0.5, -1.0, 1.0);
        const double rate = std::acos(c) / ((n_ - 1) * step_);
        rate_sq_.push_back(rate * rate);
        frames_.push_back(std::move(f));
    }
    pending_.swap(keep);
}

DelayEstimate DelayCalibrator::estimate() const {
    DelayEstimate e;
    e.frames = static_cast<int>(frames_.size());
    if (e.frames < 30) {
        e.reason = "fewer than 30 frames with a plate and full attitude coverage";
        return e;
    }
    double r2 = 0;
    for (double v : rate_sq_) r2 += v;
    e.gimbal_rate_rms = std::sqrt(r2 / rate_sq_.size());

    // Spread (RMS scatter about the mean) for every grid offset.
    std::vector<double> cost(n_);
    for (int i = 0; i < n_; ++i) {
        cv::Vec3d mean(0, 0, 0);
        for (const Frame& f : frames_) mean += f.world[i];
        mean = mean * (1.0 / frames_.size());
        double s = 0;
        for (const Frame& f : frames_) {
            const cv::Vec3d d = f.world[i] - mean;
            s += d.dot(d);
        }
        cost[i] = std::sqrt(s / frames_.size());
    }
    const int best = static_cast<int>(std::min_element(cost.begin(), cost.end()) - cost.begin());
    double d = d_min_ + best * step_;
    if (best > 0 && best < n_ - 1) {  // parabolic refinement through the 3 grid points
        const double a = cost[best - 1], b = cost[best], c = cost[best + 1];
        const double den = a - 2 * b + c;
        if (den > 0) d += 0.5 * step_ * (a - c) / den;
    }
    e.delay = d;
    e.spread = cost[best];
    const int zero = static_cast<int>(std::lround(-d_min_ / step_));
    e.spread_at_zero = (zero >= 0 && zero < n_) ? cost[zero] : cost[best];

    if (best == 0 || best == n_ - 1) {
        e.reason = "minimum at the edge of the search window: widen [d_min, d_max]";
        return e;
    }
    if (e.gimbal_rate_rms < 0.3) {
        e.reason = "gimbal barely moved (need >= 0.3 rad/s RMS): sweep it while recording";
        return e;
    }
    e.ok = true;
    return e;
}

}  // namespace autoaim
