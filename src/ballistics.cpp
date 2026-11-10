#include "autoaim/ballistics.hpp"

#include <algorithm>
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
        // sqrt(x^2+z^2), not hypot: same value to ~1 ulp for our speed range,
        // and this runs 4x per RK4 step in the hottest loop of the aimer.
        const double v = std::sqrt(s.vx * s.vx + s.vz * s.vz);
        return S{s.vx, s.vz, -k * v * s.vx, -g - k * v * s.vz};
    };
    S s{0, 0, v0 * std::cos(pitch), v0 * std::sin(pitch)};
    double t = 0;
    // RK4 with a 10 ms step and a cubic-Hermite crossing (ported from the
    // sentry review branch, docs/review_2026-11.md §4.7).  The old 1 ms step
    // with a linear crossing spent ~10x the work for less accuracy: against a
    // 10 us reference over 1-25 m and pitch -17..34 deg, the linear crossing
    // errs up to 1.2 um; the Hermite crossing (exact derivatives at both step
    // ends, which RK4 already computes) stays below 0.05 um at 10 ms.
    const double dt = 0.01;
    for (int i = 0; i < 2000; ++i) {  // 20 s of flight, as before
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
            // Cubic Hermite on [t, t+dt] using positions and velocities at both
            // ends; Newton for x(u) = s_target (x is monotone: vx > 0).
            const auto herm = [dt](double p0, double m0, double p1, double m1, double u) {
                const double u2 = u * u, u3 = u2 * u;
                return (2 * u3 - 3 * u2 + 1) * p0 + (u3 - 2 * u2 + u) * dt * m0 + (-2 * u3 + 3 * u2) * p1 +
                       (u3 - u2) * dt * m1;
            };
            const auto dherm = [dt](double p0, double m0, double p1, double m1, double u) {
                const double u2 = u * u;
                return (6 * u2 - 6 * u) * p0 + (3 * u2 - 4 * u + 1) * dt * m0 + (-6 * u2 + 6 * u) * p1 +
                       (3 * u2 - 2 * u) * dt * m1;  // d/du
            };
            double u = (s_target - s.x) / (n.x - s.x);  // linear guess
            for (int it = 0; it < 4; ++it) {
                const double du = dherm(s.x, s.vx, n.x, n.vx, u);
                if (std::abs(du) < 1e-12) break;
                u = std::clamp(u - (herm(s.x, s.vx, n.x, n.vx, u) - s_target) / du, 0.0, 1.0);
            }
            height = herm(s.z, s.vz, n.z, n.vz, u);
            t_out = t + u * dt;
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
    if (!std::isfinite(s) || !std::isfinite(h)) return sol;
    // Start from the vacuum drop compensation instead of the bare height:
    // one less fixed-point iteration on average (measured 3.0 -> 2.5).  The
    // converged solution is unchanged within the solver's own tolerance
    // (|dh| < 5e-4 m); the pitch moves by at most ~6e-5 rad.
    const double v0 = bulletSpeed();
    double aim_h = h;
    if (v0 > 0) {
        const double tv = s / v0;
        aim_h += 0.5 * cfg_.gravity * tv * tv;
    }
    // Root of f(aim_h) = h - impact_height(aim_h).  The first step is the
    // fixed-point update (aim_h += f); later steps use the secant through the
    // last two evaluations, which converges in 2-3 simulations instead of the
    // fixed point's linear rate.  Falls back to the fixed-point step whenever
    // the secant slope is degenerate.  Same 0.5 mm acceptance as before.
    // (Secant ported from the sentry review branch, docs/review_2026-11.md §4.7.)
    double prev_aim = 0, prev_f = 0;
    bool have_prev = false;
    for (int i = 0; i < 30; ++i) {
        const double pitch = std::atan2(aim_h, s);
        if (std::abs(pitch) > 1.2) return sol;
        double impact_h, t;
        if (!simulate(pitch, s, impact_h, t)) return sol;
        const double f = h - impact_h;
        if (std::abs(f) < 5e-4) {
            sol.ok = true, sol.pitch = pitch, sol.t_fly = t;
            return sol;
        }
        double next = aim_h + f;
        if (have_prev) {
            const double slope = (f - prev_f) / (aim_h - prev_aim);  // ~ -1
            if (std::isfinite(slope) && slope < -0.2 && slope > -5.0) next = aim_h - f / slope;
        }
        prev_aim = aim_h, prev_f = f, have_prev = true;
        aim_h = next;
    }
    return sol;
}

}  // namespace autoaim
