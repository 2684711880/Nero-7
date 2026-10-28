#include "nero_mid360_sim/interval_acceleration.hpp"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <iostream>
using nero_mid360_sim::IntervalAcceleration;
void require(bool value) { if (!value) { std::cerr << "IMU interval regression failed\n"; std::exit(1); } }
int main() {
  IntervalAcceleration sensor;
  require(!sensor.Sample(0., {0., 0., 0.}));
  // Contact micro-motion repeats every 5 ms. Its sampled instantaneous
  // derivative is always positive, but total delta-v each IMU period is zero.
  double integrated_delta_v = 0.;
  for (int i = 1; i <= 100; ++i) {
    const double t = i * .005;
    const double velocity = .01 * std::sin(2 * 3.141592653589793 * 200 * t);
    auto acceleration = sensor.Sample(t, {0., 0., velocity});
    require(bool(acceleration)); integrated_delta_v += (*acceleration)[2] * .005;
  }
  require(std::abs(integrated_delta_v) < 1e-10);
  sensor.Reset(0., {0., 0., 0.});
  // A real acceleration, including vertical lift acceleration, must survive.
  for (int i = 1; i <= 20; ++i) {
    const double t = i * .005;
    auto a = sensor.Sample(t, {2*t, -t, .4*t});
    require(bool(a)); require(std::abs((*a)[0]-2.) < 1e-10);
    require(std::abs((*a)[1]+1.) < 1e-10); require(std::abs((*a)[2]-.4) < 1e-10);
  }
  require(!sensor.Sample(-1., {0.,0.,0.})); // Clock reset.
  require(!sensor.Sample(2., {0.,0.,0.}));  // Missing interval.
  require(!sensor.Sample(2.005, {0.,0.,std::numeric_limits<double>::quiet_NaN()}));
  require(!sensor.Sample(2.010, {0.,0.,0.}));
  require(bool(sensor.Sample(2.015, {0.,0.,0.})));
}
