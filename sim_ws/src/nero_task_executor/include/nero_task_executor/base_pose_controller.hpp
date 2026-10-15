#pragma once
// ROS-independent reference generation and SE(2) feedback for all three new modes.
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nero_base {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Pose { double x=0, y=0, yaw=0; };
struct Velocity { double x=0, y=0, yaw=0; };
enum class Mode { Translate, Rotate, Combined };
struct Limits {
  double linear=.10, angular=.18, linear_acceleration=.15, angular_acceleration=.30;
  void validate() const {
    for (double v : {linear,angular,linear_acceleration,angular_acceleration})
      if (!std::isfinite(v) || v<=0) throw std::invalid_argument("Limits must be finite and positive");
    if(linear>.2 || angular>.4 || linear_acceleration>.4 || angular_acceleration>.8)
      throw std::invalid_argument("Limits exceed the simulation adapter envelope");
  }
};
struct Reference { Pose pose; Velocity world_velocity; double progress=0; bool finished=false; };

class Trajectory {
public:
  Trajectory(Mode mode, Pose start, Pose requested, Limits limits, double duration=0)
    : start_(start), goal_(requested), limits_(limits) {
    limits_.validate();
    for(double v:{start.x,start.y,start.yaw,requested.x,requested.y,requested.yaw,duration})
      if(!std::isfinite(v))throw std::invalid_argument("Non-finite trajectory parameter");
    if(mode==Mode::Translate)goal_.yaw=start.yaw;
    if(mode==Mode::Rotate){goal_.x=start.x;goal_.y=start.y;}
    delta_={goal_.x-start.x,goal_.y-start.y,wrap(goal_.yaw-start.yaw)};
    goal_.yaw=wrap(start_.yaw+delta_.yaw);
    const double distance=std::hypot(delta_.x,delta_.y), angle=std::abs(delta_.yaw);
    // Quintic phase has max s'=1.875, max |s''|=10/sqrt(3).
    // Reserve 30% of velocity limits for tracking corrections.
    const double minimum=std::max({.5,1.875*distance/(.7*limits.linear),
        1.875*angle/(.7*limits.angular),std::sqrt(5.7735026919*distance/limits.linear_acceleration),
        std::sqrt(5.7735026919*angle/limits.angular_acceleration)});
    if(duration<0 || (duration>0 && duration+1e-9<minimum))
      throw std::invalid_argument("duration_s is too short for the requested displacement and limits");
    duration_=duration>0?duration:minimum;
  }
  Reference sample(double elapsed) const {
    const double u=std::clamp(elapsed/duration_,0.,1.);
    const double s=u*u*u*(10+u*(-15+6*u));
    const double ds=30*u*u*(1-u)*(1-u)/duration_;
    return {{start_.x+s*delta_.x,start_.y+s*delta_.y,wrap(start_.yaw+s*delta_.yaw)},
            {ds*delta_.x,ds*delta_.y,ds*delta_.yaw},s,elapsed>=duration_};
  }
  Pose goal() const {return goal_;}
  double duration() const {return duration_;}
  Limits limits() const {return limits_;}
private:
  Pose start_,goal_,delta_;Limits limits_;double duration_;
};

inline bool inside(Pose current,Pose target,double position=.012,double angle=.015) {
  return std::hypot(target.x-current.x,target.y-current.y)<position &&
         std::abs(wrap(target.yaw-current.yaw))<angle;
}

inline double smoothstep01(double u) {
  u=std::clamp(u,0.,1.);
  return u*u*(3.-2.*u);
}

// Desired velocity is in odom; Twist is in base_link. Both components are allowed
// at the same time, including when the heading error is large.
inline Velocity feedback(const Reference& reference,Pose actual,Limits limits) {
  double vx=reference.world_velocity.x+1.2*(reference.pose.x-actual.x);
  double vy=reference.world_velocity.y+1.2*(reference.pose.y-actual.y);
  double wz=reference.world_velocity.yaw+1.2*wrap(reference.pose.yaw-actual.yaw);
  const double error=std::hypot(reference.pose.x-actual.x,reference.pose.y-actual.y);
  const double angle=wrap(reference.pose.yaw-actual.yaw);
  double speed=std::hypot(vx,vy);
  // Docking stiction compensation is inappropriate during cruise: forcing
  // independent linear/yaw minima changes curvature and makes steering hunt.
  // Blend it out once either reference component represents useful motion.
  const double reference_activity=std::max(std::hypot(reference.world_velocity.x,
      reference.world_velocity.y)/.02,std::abs(reference.world_velocity.yaw)/.06);
  const double docking_weight=1.-smoothstep01(reference_activity);
  // Ramp compensation continuously instead of switching at 8 mm / 0.008 rad.
  // Full docking drive is retained outside the inner arrival tolerances.
  // The second ramp also keeps the command continuous through zero when
  // feedforward and feedback cancel; compensation must preserve their sum's sign.
  if(speed>1e-9) {
    const double floor=docking_weight*std::min(.04,limits.linear)*smoothstep01(error/.012)*smoothstep01(speed/.01);
    if(speed<floor){vx*=floor/speed;vy*=floor/speed;speed=floor;}
  }
  const double angular_floor=docking_weight*std::min(.12,limits.angular)*smoothstep01(std::abs(angle)/.015)*smoothstep01(std::abs(wz)/.01);
  if(std::abs(wz)<angular_floor)wz=std::copysign(angular_floor,wz);
  if(speed>limits.linear){vx*=limits.linear/speed;vy*=limits.linear/speed;}
  wz=std::clamp(wz,-limits.angular,limits.angular);
  const double c=std::cos(actual.yaw),s=std::sin(actual.yaw);
  return {c*vx+s*vy,-s*vx+c*vy,wz};
}

inline Velocity slew(Velocity previous,Velocity target,Limits limits,double dt) {
  dt=std::clamp(dt,0.,.1);
  const double dx=target.x-previous.x,dy=target.y-previous.y,length=std::hypot(dx,dy);
  const double fraction=length>0?std::min(1.,limits.linear_acceleration*dt/length):0.;
  return {previous.x+fraction*dx,previous.y+fraction*dy,
          previous.yaw+std::clamp(target.yaw-previous.yaw,
              -limits.angular_acceleration*dt,limits.angular_acceleration*dt)};
}
}  // namespace nero_base
