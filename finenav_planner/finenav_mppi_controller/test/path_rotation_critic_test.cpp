// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "finenav_mppi_controller/critics/path_rotation_critic.hpp"

namespace
{

class TestMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return false;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.0F;}
  int getCost(const Position3D &) const override {return 0;}
  float costAtPose(float, float, float) const override {return 0.0F;}
  std::string getBaseFrameID() const override {return "map";}
};

geometry_msgs::msg::Quaternion quaternionFromYaw(const float yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5F);
  orientation.w = std::cos(yaw * 0.5F);
  return orientation;
}

class PathRotationCriticTest : public testing::Test
{
protected:
  void SetUp() override
  {
    state_.reset(4, 3);
    trajectories_.reset(4, 3);
    path_.reset(1);
    costs_.resize(4);
    costs_.setZero();

    finenav_mppi_controller::Params params;
    params.PathRotationCritic.enabled = true;
    params.PathRotationCritic.cost_power = 1;
    params.PathRotationCritic.cost_weight = 8.0;
    critic_.initialize(params);
  }

  void score()
  {
    mppi::CriticData data{state_, trajectories_, path_, costs_, model_dt_, false};
    critic_.score(data, map_view_);
  }

  mppi::models::State state_;
  mppi::models::Trajectories trajectories_;
  mppi::models::Path path_;
  mppi::models::Vector costs_;
  float model_dt_{0.05F};
  TestMapView map_view_;
  mppi::critics::PathRotationCritic critic_;
};

TEST_F(PathRotationCriticTest, PenalizesOppositeRotationIndependentOfAngularSpeed)
{
  state_.pose.pose.orientation = quaternionFromYaw(0.0F);
  path_.yaws(0) = static_cast<float>(M_PI_2);

  state_.wz.row(0) << 0.1F, 0.5F, 1.5F;
  state_.wz.row(1) << -0.1F, -0.1F, -0.1F;
  state_.wz.row(2) << -0.2F, -1.0F, -1.8F;
  state_.wz.row(3).setZero();

  score();

  EXPECT_FLOAT_EQ(costs_(0), 0.0F);
  EXPECT_FLOAT_EQ(costs_(1), 8.0F);
  EXPECT_FLOAT_EQ(costs_(2), 8.0F);
  EXPECT_FLOAT_EQ(costs_(3), 0.0F);
}

TEST_F(PathRotationCriticTest, IgnoresTrajectoriesContainingNonzeroLinearVelocity)
{
  state_.pose.pose.orientation = quaternionFromYaw(0.0F);
  path_.yaws(0) = static_cast<float>(M_PI_2);
  state_.wz.setConstant(-1.0F);

  state_.vx(0, 1) = 0.01F;
  state_.vy(1, 2) = -0.01F;

  score();

  EXPECT_FLOAT_EQ(costs_(0), 0.0F);
  EXPECT_FLOAT_EQ(costs_(1), 0.0F);
  EXPECT_FLOAT_EQ(costs_(2), 8.0F);
  EXPECT_FLOAT_EQ(costs_(3), 8.0F);
}

TEST_F(PathRotationCriticTest, UsesClosestPathHeadingAndShortestRotationDirection)
{
  path_.reset(2);
  path_.x << 0.0F, 10.0F;
  path_.y << 0.0F, 0.0F;
  path_.yaws << 1.0F, 3.0F;
  state_.pose.pose.position.x = 9.5;
  state_.pose.pose.orientation = quaternionFromYaw(-3.0F);

  state_.wz.row(0).setConstant(-0.4F);
  state_.wz.row(1).setConstant(0.4F);

  score();

  EXPECT_FLOAT_EQ(costs_(0), 0.0F);
  EXPECT_FLOAT_EQ(costs_(1), 8.0F);
}

TEST_F(PathRotationCriticTest, DoesNotPreferEitherDirectionWhenHeadingIsAligned)
{
  state_.pose.pose.orientation = quaternionFromYaw(0.75F);
  path_.yaws(0) = 0.75F;
  state_.wz.row(0).setConstant(-0.5F);
  state_.wz.row(1).setConstant(0.5F);

  score();

  EXPECT_FLOAT_EQ(costs_(0), 0.0F);
  EXPECT_FLOAT_EQ(costs_(1), 0.0F);
}

}  // namespace
