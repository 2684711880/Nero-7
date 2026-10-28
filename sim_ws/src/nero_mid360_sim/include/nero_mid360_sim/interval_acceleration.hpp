#pragma once
#include <array>
#include <cmath>
#include <optional>

namespace nero_mid360_sim {
// Average world acceleration over the actual IMU integration interval.
// Telescoping velocity differences preserve delta-v; decimating instantaneous
// contact-solver accelerations can alias a zero-mean impulse into a false bias.
class IntervalAcceleration {
 public:
  using Vector = std::array<double, 3>;
  void Reset(double time, const Vector &velocity) {
    valid_ = std::isfinite(time);
    for (double v : velocity) valid_ = valid_ && std::isfinite(v);
    time_ = time; velocity_ = velocity;
  }
  std::optional<Vector> Sample(double time, const Vector &velocity) {
    const double dt = time - time_;
    bool usable = valid_ && std::isfinite(time) && dt > 0 && dt <= .1;
    for (double v : velocity) usable = usable && std::isfinite(v);
    Vector acceleration{};
    if (usable) {
      for (std::size_t i = 0; i < 3; ++i) acceleration[i] = (velocity[i] - velocity_[i]) / dt;
    }
    Reset(time, velocity);
    if (!usable) return std::nullopt;
    return acceleration;
  }
 private:
  bool valid_ = false;
  double time_ = 0.;
  Vector velocity_{};
};
}  // namespace nero_mid360_sim
