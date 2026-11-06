#include <gtest/gtest.h>

#include "finenav_mppi_controller/motion_models.hpp"

TEST(MotionModelVelocityProjection, StopsForwardOverspeedRegardlessOfRequest)
{
  constexpr float min_velocity = 0.0F;
  constexpr float max_velocity = 0.4F;
  constexpr float min_acceleration = -3.0F;
  constexpr float max_acceleration = 3.0F;
  constexpr float dt = 0.05F;

  for (const float request : {0.0F, 0.4F, 2.0F}) {
    EXPECT_NEAR(
      mppi::MotionModel::projectVelocity(
        request, 0.58F, min_velocity, max_velocity,
        min_acceleration, max_acceleration, dt),
      0.0F, 1e-6F);
  }
}

TEST(MotionModelVelocityProjection, StopsReverseOverspeedRegardlessOfRequest)
{
  for (const float request : {-2.0F, -0.4F, 0.4F}) {
    EXPECT_NEAR(
      mppi::MotionModel::projectVelocity(
        request, -0.58F, -0.4F, 0.4F, -3.0F, 3.0F, 0.05F),
      0.0F, 1e-6F);
  }
}

TEST(MotionModelVelocityProjection, KeepsNormalProjectionInsideVelocityEnvelope)
{
  EXPECT_NEAR(
    mppi::MotionModel::projectVelocity(2.0F, 0.3F, 0.0F, 0.4F, -3.0F, 3.0F, 0.05F),
    0.4F, 1e-6F);
}

TEST(MotionModelVelocityProjection, StopCheckUsesApplicableVelocityAxes)
{
  mppi::models::ControlConstraints constraints;
  constraints.vx_min = -0.4F;
  constraints.vx_max = 0.4F;
  constraints.vy = 0.4F;
  constraints.wz = 0.4F;
  constraints.ax_min = -3.0F;
  constraints.ax_max = 3.0F;
  constraints.ay_min = -3.0F;
  constraints.ay_max = 3.0F;
  constraints.az_max = 3.0F;

  mppi::DiffDriveMotionModel diff_drive;
  diff_drive.setConstraints(constraints, 0.05F);
  EXPECT_FALSE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.58F, 0.0F));
  EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.58F, 0.0F, 0.0F));
  EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(-0.58F, 0.0F, 0.0F));
  EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.0F, 0.58F));
  EXPECT_TRUE(diff_drive.shouldStopForVelocityLimits(0.0F, 0.0F, -0.58F));
  EXPECT_FALSE(diff_drive.shouldStopForVelocityLimits(0.55F, 0.0F, 0.55F));

  mppi::OmniMotionModel omni;
  omni.setConstraints(constraints, 0.05F);
  EXPECT_TRUE(omni.shouldStopForVelocityLimits(0.0F, 0.58F, 0.0F));
  EXPECT_TRUE(omni.shouldStopForVelocityLimits(0.0F, -0.58F, 0.0F));
}
