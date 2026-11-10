#include "autoaim/ballistics.hpp"

#include <cmath>

namespace autoaim {

Ballistics::Ballistics(const BallisticConfig& cfg) : cfg_(cfg) { setBulletClass(cfg.bullet_mm); }

double Ballistics::bulletSpeed() const {
    if (speed_override_ > 0) return speed_override_;
    return mm_ == 42 ? cfg_.bullet_speed_42 : cfg_.bullet_speed;
}

bool Ballistics::simulate(double pitch, double s_target, double& height, double& t_out) const {
    const double v0 = bulletSpeed(), g = cfg_.gravity;
    const double k = cfg_.use_drag ? dragK() : 0.0;
    if (s_target <= 0) {
        height = 0, t_out = 0;
        return true;
    }
    if (k <= 0) {  // closed form
        const double vx = v0 * std::cos(pitch), vz = v0 * std::sin(pitch);
        if (vx <= 1e-6) return false;
        const double t = s_target / vx;
        height = vz * t - 0.5 * g * t * t;
        t_out = t;
        return true;
    }
    struct S {
        double x, z, vx, vz;
    };
    auto deriv = [&](const S& s) {
        const double v = std::hypot(s.vx, s.vz);
        return S{s.vx, s.vz, -k * v * s.vx, -g - k * v * s.vz};
    };
    S s{0, 0, v0 * std::cos(pitch), v0 * std::sin(pitch)};
    double t = 0;
    const double dt = 0.001;
    for (int i = 0; i < 20000; ++i) {
        const S k1 = deriv(s);
        const S k2 = deriv({s.x + 0.5 * dt * k1.x, s.z + 0.5 * dt * k1.z, s.vx + 0.5 * dt * k1.vx,
                            s.vz + 0.5 * dt * k1.vz});
        const S k3 = deriv({s.x + 0.5 * dt * k2.x, s.z + 0.5 * dt * k2.z, s.vx + 0.5 * dt * k2.vx,
                            s.vz + 0.5 * dt * k2.vz});
        const S k4 = deriv({s.x + dt * k3.x, s.z + dt * k3.z, s.vx + dt * k3.vx, s.vz + dt * k3.vz});
        S n{s.x + dt / 6 * (k1.x + 2 * k2.x + 2 * k3.x + k4.x),
            s.z + dt / 6 * (k1.z + 2 * k2.z + 2 * k3.z + k4.z),
            s.vx + dt / 6 * (k1.vx + 2 * k2.vx + 2 * k3.vx + k4.vx),
            s.vz + dt / 6 * (k1.vz + 2 * k2.vz + 2 * k3.vz + k4.vz)};
        if (n.x >= s_target) {
            const double a = (s_target - s.x) / (n.x - s.x);
            height = s.z + a * (n.z - s.z);
            t_out = t + a * dt;
            return true;
        }
        if (n.vx <= 1e-3) return false;
        s = n;
        t += dt;
    }
    return false;
}

BallisticSolution Ballistics::solve(double s, double h) const {
    BallisticSolution sol;
    double aim_h = h;
    for (int i = 0; i < 30; ++i) {
        const double pitch = std::atan2(aim_h, s);
        if (std::abs(pitch) > 1.2) return sol;
        double impact_h, t;
        if (!simulate(pitch, s, impact_h, t)) return sol;
        const double dh = h - impact_h;
        if (std::abs(dh) < 5e-4) {
            sol.ok = true, sol.pitch = pitch, sol.t_fly = t;
            return sol;
        }
        aim_h += dh;
    }
    return sol;
}

}  // namespace autoaim
