#include <gtest/gtest.h>
#include <nero_task_executor/base_pose_controller.hpp>
using namespace nero_base;
TEST(BaseReference, TranslateHoldsCapturedHeading) {
  Trajectory path(Mode::Translate,{1,2,.7},{2,3,-2},{});
  for(int i=0;i<=100;++i)EXPECT_NEAR(path.sample(path.duration()*i/100).pose.yaw,.7,1e-12);
  EXPECT_NEAR(path.goal().x,2,1e-12);
}
TEST(BaseReference, RotateHoldsCapturedPosition) {
  Trajectory path(Mode::Rotate,{1,2,.7},{20,30,1.7},{});
  for(int i=0;i<=100;++i){auto r=path.sample(path.duration()*i/100);EXPECT_DOUBLE_EQ(r.pose.x,1);EXPECT_DOUBLE_EQ(r.pose.y,2);}
}
TEST(BaseReference, CombinedSharesProgressAndHasSmoothEndpoints) {
  Trajectory path(Mode::Combined,{0,0,0},{1,.5,1},{});
  const auto first=path.sample(0),middle=path.sample(path.duration()/2),last=path.sample(path.duration());
  EXPECT_NEAR(middle.pose.x,.5,1e-12);EXPECT_NEAR(middle.pose.y,.25,1e-12);EXPECT_NEAR(middle.pose.yaw,.5,1e-12);
  EXPECT_GT(middle.world_velocity.x,0);EXPECT_GT(middle.world_velocity.yaw,0);
  EXPECT_DOUBLE_EQ(first.world_velocity.x,0);EXPECT_DOUBLE_EQ(first.world_velocity.yaw,0);
  EXPECT_DOUBLE_EQ(last.world_velocity.x,0);EXPECT_DOUBLE_EQ(last.world_velocity.yaw,0);
  EXPECT_TRUE(last.finished);EXPECT_FALSE(middle.finished);
}
TEST(BaseReference, WrapUsesShortRotation) {
  Trajectory path(Mode::Rotate,{0,0,179*pi/180},{0,0,-179*pi/180},{});
  EXPECT_NEAR(wrap(path.sample(path.duration()/2).pose.yaw-179*pi/180),pi/180,1e-9);
}
TEST(BaseReference, SampledVelocityAndAccelerationRespectLimits) {
  Limits limits;Trajectory path(Mode::Combined,{0,0,0},{.4,.2,1.2},limits);
  const double dt=path.duration()/10000;auto previous=path.sample(0).world_velocity;
  for(int i=1;i<=10000;++i) {
    auto v=path.sample(i*dt).world_velocity;
    EXPECT_LE(std::hypot(v.x,v.y),limits.linear+1e-8);EXPECT_LE(std::abs(v.yaw),limits.angular+1e-8);
    EXPECT_LE(std::hypot(v.x-previous.x,v.y-previous.y)/dt,limits.linear_acceleration+1e-6);
    EXPECT_LE(std::abs(v.yaw-previous.yaw)/dt,limits.angular_acceleration+1e-6);previous=v;
  }
}
TEST(BaseReference, RejectsImpossibleDurationAndInvalidLimits) {
  EXPECT_THROW(Trajectory(Mode::Combined,{},{1,1,1},{},.01),std::invalid_argument);
  Limits limits;limits.linear=0;EXPECT_THROW(Trajectory(Mode::Combined,{},{1,1,1},limits),std::invalid_argument);
}
TEST(BaseFeedback, WorldVelocityTransformsIntoBodyFrame) {
  Reference r;r.pose={0,0,pi/2};r.world_velocity={.1,0,0};const auto v=feedback(r,{0,0,pi/2},{});
  EXPECT_NEAR(v.x,0,1e-12);EXPECT_NEAR(v.y,-.1,1e-12);EXPECT_NEAR(v.yaw,0,1e-12);
}
TEST(BaseFeedback, LargeHeadingErrorDoesNotBlockTranslation) {
  Reference r;r.pose={.1,0,pi/2};r.world_velocity={.05,0,.1};const auto v=feedback(r,{},{});
  EXPECT_GT(v.x,0);EXPECT_GT(v.yaw,0);
}
TEST(BaseFeedback, SlewBoundsAcceleration) {
  Limits limits;const auto v=slew({},{.1,.1,.18},limits,.02);
  EXPECT_LE(std::hypot(v.x,v.y),limits.linear_acceleration*.02+1e-12);
  EXPECT_LE(std::abs(v.yaw),limits.angular_acceleration*.02+1e-12);
}
TEST(BaseFeedback, GoalToleranceWrapsHeading) {
  EXPECT_TRUE(inside({0,0,pi-.001},{0,0,-pi+.001}));EXPECT_FALSE(inside({.1,0,0},{}));
}
TEST(BaseFeedback, TrackingCompensationNeverReversesTheRequestedAngularVelocity) {
  // Reference still rotates clockwise while measured yaw is slightly ahead.
  // A positive pose correction must not reverse the negative total command.
  Reference r;r.pose={0,0,.01};r.world_velocity={0,0,-.1};
  const auto v=feedback(r,{},{});
  EXPECT_LT(v.yaw,0);
  r.pose.yaw=-.01;r.world_velocity.yaw=.1;
  EXPECT_GT(feedback(r,{},{}).yaw,0);
}
TEST(BaseFeedback, CompensationIsContinuousAcrossOldEightMillimeterBoundary) {
  Reference before;before.pose={.00799,0,.00799};before.world_velocity={.01,0,.04};
  auto after=before;after.pose={.00801,0,.00801};
  const auto a=feedback(before,{},{}), b=feedback(after,{},{});
  EXPECT_LT(std::abs(a.x-b.x),.001);
  EXPECT_LT(std::abs(a.yaw-b.yaw),.001);
}
TEST(BaseFeedback, DockingStillHasDriveAboveInnerArrivalTolerance) {
  Reference r;r.pose={.027,0,.016};
  const auto v=feedback(r,{},{});
  EXPECT_GE(v.x,.04-1e-9);EXPECT_GE(v.yaw,.12-1e-9);
  Limits limits;limits.linear=.02;limits.angular=.05;
  const auto limited=feedback(r,{},limits);
  EXPECT_LE(limited.x,.02);EXPECT_LE(limited.yaw,.05);
}
TEST(BaseFeedback, CompensationIsContinuousWhereFeedforwardAndFeedbackCancel) {
  Reference r;r.pose={.02,0,.02};r.world_velocity={-.024,0,-.024};
  const auto zero=feedback(r,{},{});
  EXPECT_NEAR(zero.x,0,1e-10);EXPECT_NEAR(zero.yaw,0,1e-10);
  r.world_velocity={-.024+1e-6,0,-.024+1e-6};
  const auto nearby=feedback(r,{},{});
  EXPECT_LT(std::abs(nearby.x),1e-4);EXPECT_LT(std::abs(nearby.yaw),1e-4);
}
TEST(BaseFeedback, CruisingReferenceDoesNotForceDockingMinimumSpeeds) {
  Reference r;r.pose={0,0,.012};r.world_velocity={.04,0,.04};
  const auto v=feedback(r,{},{});
  EXPECT_NEAR(v.x,.04,1e-10);
  EXPECT_NEAR(v.yaw,.04+1.2*.012,1e-10);
}
