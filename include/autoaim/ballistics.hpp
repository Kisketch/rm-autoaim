// Projectile model and pitch solving.
//
// rmoss_core / FYT2024 use closed-form approximations (pure gravity, or the
// one-dimensional "x = ln(1 + k v t) / k" drag model). Here the trajectory is
// integrated numerically (RK4) with full quadratic drag  a = -g z - k |v| v,
// which stays accurate for steep shots and long ranges; pitch is solved by a
// fixed-point iteration on the aim height (same scheme as FYT2024).
#pragma once

#include "autoaim/config.hpp"

namespace autoaim {

struct BallisticSolution {
    bool ok = false;
    double pitch = 0;  // rad, positive up
    double t_fly = 0;  // s
};

class Ballistics {
public:
    explicit Ballistics(const BallisticConfig& cfg);
    // Height and time of flight when the bullet reaches horizontal range s.
    bool simulate(double pitch, double s, double& height, double& t) const;
    // Pitch that hits (s, h); s horizontal range, h height (metres).
    BallisticSolution solve(double s, double h) const;
    const BallisticConfig& config() const { return cfg_; }

    // Active projectile class (17 or 42) and its parameters.  The class is
    // initialised from cfg.bullet_mm and can be switched at runtime.
    int bulletMm() const { return mm_; }
    void setBulletClass(int mm) { mm_ = mm == 42 ? 42 : 17; }
    // Muzzle velocity actually in use: the measured override when one is
    // set, otherwise the active class's config value.
    double bulletSpeed() const;
    double dragK() const { return mm_ == 42 ? cfg_.drag_k_42 : cfg_.drag_k; }
    // Feed the measured muzzle velocity (referee system, muzzle sensor).
    // v <= 0 clears the override and falls back to the config value.
    void setMeasuredBulletSpeed(double v) { speed_override_ = v > 0 ? v : 0.0; }
    bool hasMeasuredSpeed() const { return speed_override_ > 0; }

private:
    BallisticConfig cfg_;
    int mm_ = 17;
    double speed_override_ = 0.0;
};

}  // namespace autoaim
