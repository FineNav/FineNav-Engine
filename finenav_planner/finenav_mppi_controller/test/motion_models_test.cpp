#include <gtest/gtest.h>

#include "finenav_mppi_controller/motion_models.hpp"

TEST(MotionModelVelocityProjection, BrakesForwardOverspeedRegardlessOfRequest)
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
      0.43F, 1e-6F);
  }
}

TEST(MotionModelVelocityProjection, BrakesReverseOverspeedRegardlessOfRequest)
{
  for (const float request : {-2.0F, -0.4F, 0.4F}) {
    EXPECT_NEAR(
      mppi::MotionModel::projectVelocity(
        request, -0.58F, -0.4F, 0.4F, -3.0F, 3.0F, 0.05F),
      -0.43F, 1e-6F);
  }
}

TEST(MotionModelVelocityProjection, KeepsNormalProjectionInsideVelocityEnvelope)
{
  EXPECT_NEAR(
    mppi::MotionModel::projectVelocity(2.0F, 0.3F, 0.0F, 0.4F, -3.0F, 3.0F, 0.05F),
    0.4F, 1e-6F);
}
