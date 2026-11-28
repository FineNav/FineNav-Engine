// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "finenav_mppi_controller/controller.hpp"
#include "finenav_mppi_controller/optimizer.hpp"

namespace
{

class TestMapView final : public nav2_mppi_controller::IMapView
{
public:
  bool isTrackingUnknown() const override {return false;}
  bool considerFootprint() const override {return true;}
  bool isCollision(float, float, float) const override {return false;}
  float getRadius() const override {return 0.5F;}
  int getCost(const Position3D &) const override {return 0;}
  float costAtPose(float, float, float) const override {return 0.0F;}
  std::string getBaseFrameID() const override {return "map";}
};

geometry_msgs::msg::PoseStamped makePose(double x, double y = 0.0, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw * 0.5);
  pose.pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

finenav_mppi_controller::Params makeParams()
{
  finenav_mppi_controller::Params params;
  params.batch_size = 8;
  params.time_steps = 20;
  params.iteration_count = 1;
  params.model_dt = 0.1;
  params.min_time_steps = 5;
  params.adaptive_horizon_distance = 1.4;
  params.adaptive_horizon_linear_vel = 0.25;
  params.adaptive_horizon_angular_vel = 0.5;
  params.regenerate_noises = false;
  params.GoalAngleCritic.enabled = false;
  return params;
}

nav_msgs::msg::Path makePath(std::initializer_list<geometry_msgs::msg::PoseStamped> poses)
{
  nav_msgs::msg::Path path;
  path.poses.assign(poses.begin(), poses.end());
  return path;
}

class TestOptimizer : public mppi::Optimizer
{
public:
  unsigned int activeSteps() const {return active_time_steps_;}

  const mppi::models::ControlSequence & controlSequence() const {return control_sequence_;}

  void prepareCycle(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & path)
  {
    prepare(pose, speed, path);
  }

  void fillWarmStart(float vx, float vy, float wz)
  {
    control_sequence_.vx.setConstant(vx);
    control_sequence_.vy.setConstant(vy);
    control_sequence_.wz.setConstant(wz);
  }
};

class RetryOnceCritic : public mppi::critics::CriticFunction
{
public:
  void initialize(const finenav_mppi_controller::Params &) override {}

  void score(mppi::CriticData & data, const nav2_mppi_controller::IMapView &) override
  {
    observed_steps.push_back(data.trajectories.x.cols());
    data.fail_flag = observed_steps.size() == 1;
  }

  std::vector<Eigen::Index> observed_steps;
};

TEST(AdaptiveHorizon, UsesDistanceToGoalAndKeepsFullHorizonFarAway)
{
  auto params = makeParams();
  TestOptimizer optimizer;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  const auto robot = makePose(0.0);
  const geometry_msgs::msg::Twist speed;

  const auto nearby_goal_with_long_path = makePath({robot, makePose(10.0), makePose(0.3)});
  const auto near_result = optimizer.evalControl(robot, speed, nearby_goal_with_long_path, map_view);
  EXPECT_EQ(near_result.controls.rows(), 12);
  EXPECT_EQ(near_result.trajectory.rows(), 12);
  EXPECT_EQ(optimizer.activeSteps(), 12U);

  const auto far_path = makePath({robot, makePose(1.5)});
  const auto far_result = optimizer.evalControl(robot, speed, far_path, map_view);
  EXPECT_EQ(far_result.controls.rows(), params.time_steps);
  EXPECT_EQ(optimizer.activeSteps(), static_cast<unsigned int>(params.time_steps));

  params.adaptive_horizon = false;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  const auto fixed_result = optimizer.evalControl(
    robot, speed, makePath({makePose(0.3)}), map_view);
  EXPECT_EQ(fixed_result.controls.rows(), params.time_steps);
}

TEST(AdaptiveHorizon, GoalAngleActivationFollowsCriticEnableAndDistanceThreshold)
{
  auto params = makeParams();
  params.time_steps = 40;
  params.adaptive_horizon_distance = 1.0;
  params.GoalAngleCritic.enabled = true;
  params.GoalAngleCritic.threshold_to_consider = 0.5;
  TestOptimizer optimizer;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  const geometry_msgs::msg::Twist speed;

  auto goal = makePose(0.1, 0.0, M_PI_2);
  auto result = optimizer.evalControl(makePose(0.0), speed, makePath({goal}), map_view);
  EXPECT_EQ(result.controls.rows(), 32);

  params.GoalAngleCritic.enabled = false;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  result = optimizer.evalControl(makePose(0.0), speed, makePath({goal}), map_view);
  EXPECT_EQ(result.controls.rows(), static_cast<Eigen::Index>(params.min_time_steps));

  params.GoalAngleCritic.enabled = true;
  params.GoalAngleCritic.threshold_to_consider = 0.75;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  goal = makePose(0.6, 0.0, M_PI_2);
  result = optimizer.evalControl(makePose(0.0), speed, makePath({goal}), map_view);
  EXPECT_EQ(result.controls.rows(), 32);

  params.GoalAngleCritic.threshold_to_consider = 0.5;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  goal = makePose(0.5, 0.0, M_PI_2);
  result = optimizer.evalControl(makePose(0.0), speed, makePath({goal}), map_view);
  EXPECT_EQ(result.controls.rows(), 20);
}

TEST(AdaptiveHorizon, BrakingTimeProvidesMinimumHorizon)
{
  auto params = makeParams();
  params.ax_min = -0.5;
  TestOptimizer optimizer;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  geometry_msgs::msg::Twist speed;
  speed.linear.x = 0.45;

  const auto result = optimizer.evalControl(
    makePose(0.0), speed, makePath({makePose(0.02)}), map_view);

  EXPECT_EQ(result.controls.rows(), 9);
}

TEST(AdaptiveHorizon, ShorterHorizonRaisesFirstForwardCommandNearGoal)
{
  auto params = makeParams();
  params.batch_size = 1024;
  params.iteration_count = 3;
  params.temperature = 0.1;
  params.vx_min = 0.0;
  params.GoalCritic.cost_weight = 15.0;
  params.VelocityDeadbandCritic.enabled = true;
  params.GoalAngleCritic.enabled = false;
  const auto robot = makePose(0.0);
  const auto path = makePath({makePose(0.04)});
  const geometry_msgs::msg::Twist speed;
  TestMapView map_view;

  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> adaptive_critics;
  adaptive_critics.push_back(std::make_unique<mppi::critics::GoalCritic>());
  adaptive_critics.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());
  TestOptimizer adaptive_optimizer;
  adaptive_optimizer.initialize("adaptive_horizon_test", adaptive_critics, params);
  const auto adaptive_result = adaptive_optimizer.evalControl(robot, speed, path, map_view);

  params.adaptive_horizon = false;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> fixed_critics;
  fixed_critics.push_back(std::make_unique<mppi::critics::GoalCritic>());
  fixed_critics.push_back(std::make_unique<mppi::critics::VelocityDeadbandCritic>());
  TestOptimizer fixed_optimizer;
  fixed_optimizer.initialize("adaptive_horizon_test", fixed_critics, params);
  const auto fixed_result = fixed_optimizer.evalControl(robot, speed, path, map_view);

  ASSERT_EQ(adaptive_result.controls.rows(), params.min_time_steps);
  ASSERT_EQ(fixed_result.controls.rows(), params.time_steps);
  EXPECT_GT(adaptive_result.controls(0, 0), fixed_result.controls(0, 0));
}

TEST(AdaptiveHorizon, ResizingResetAndFallbackPreserveActiveHorizonAndClearTail)
{
  auto params = makeParams();
  params.motion_model = "Omni";
  params.vy_max = 0.5;
  TestOptimizer optimizer;
  auto retry_critic = std::make_unique<RetryOnceCritic>();
  auto * retry_observer = retry_critic.get();
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  critics.push_back(std::move(retry_critic));
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  const auto robot = makePose(0.0);
  const geometry_msgs::msg::Twist speed;
  const auto near_path = makePath({makePose(0.02)});
  const auto far_path = makePath({robot, makePose(2.0)});

  optimizer.fillWarmStart(0.4F, -0.2F, 0.1F);
  auto result = optimizer.evalControl(robot, speed, near_path, map_view);
  ASSERT_EQ(result.controls.rows(), params.min_time_steps);
  EXPECT_EQ(optimizer.controlSequence().vx.size(), params.time_steps);
  EXPECT_TRUE(optimizer.controlSequence().vx.tail(params.time_steps - params.min_time_steps).isZero());
  EXPECT_TRUE(optimizer.controlSequence().vy.tail(params.time_steps - params.min_time_steps).isZero());
  EXPECT_TRUE(optimizer.controlSequence().wz.tail(params.time_steps - params.min_time_steps).isZero());
  ASSERT_EQ(retry_observer->observed_steps.size(), 2U);
  EXPECT_EQ(retry_observer->observed_steps[0], params.min_time_steps);
  EXPECT_EQ(retry_observer->observed_steps[1], params.min_time_steps);

  optimizer.prepareCycle(robot, speed, far_path);
  EXPECT_EQ(optimizer.activeSteps(), static_cast<unsigned int>(params.time_steps));
  EXPECT_TRUE(optimizer.controlSequence().vx.tail(params.time_steps - params.min_time_steps).isZero());
  EXPECT_TRUE(optimizer.controlSequence().vy.tail(params.time_steps - params.min_time_steps).isZero());
  EXPECT_TRUE(optimizer.controlSequence().wz.tail(params.time_steps - params.min_time_steps).isZero());

  result = optimizer.evalControl(robot, speed, far_path, map_view);
  EXPECT_EQ(result.controls.rows(), params.time_steps);
  EXPECT_EQ(optimizer.activeSteps(), static_cast<unsigned int>(params.time_steps));
  optimizer.reset();
  EXPECT_EQ(optimizer.activeSteps(), static_cast<unsigned int>(params.time_steps));
  EXPECT_TRUE(optimizer.getControlSequence().isZero());

  result = optimizer.evalControl(robot, speed, near_path, map_view);
  EXPECT_EQ(result.controls.rows(), params.min_time_steps);
}

TEST(AdaptiveHorizon, MinimumAboveMaximumClampsToMaximumAndDirectInitGuardsZero)
{
  auto params = makeParams();
  params.min_time_steps = params.time_steps + 10;
  TestOptimizer optimizer;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  geometry_msgs::msg::Twist speed;

  auto result = optimizer.evalControl(
    makePose(0.0), speed, makePath({makePose(0.1)}), map_view);
  EXPECT_EQ(result.controls.rows(), params.time_steps);

  for (const std::int64_t invalid_minimum : {0, -5}) {
    params.min_time_steps = invalid_minimum;
    optimizer.initialize("adaptive_horizon_test", critics, params);
    result = optimizer.evalControl(makePose(0.0), speed, makePath({makePose(0.02)}), map_view);
    EXPECT_EQ(result.controls.rows(), 1);
  }
}

TEST(AdaptiveHorizon, VelocityLimitStopReturnsOnlyTheActiveZeroSequence)
{
  auto params = makeParams();
  TestOptimizer optimizer;
  std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
  optimizer.initialize("adaptive_horizon_test", critics, params);
  TestMapView map_view;
  geometry_msgs::msg::Twist speed;
  speed.angular.z = 2.5;

  const auto result = optimizer.evalControl(
    makePose(0.0), speed, makePath({makePose(0.02)}), map_view);

  EXPECT_EQ(result.controls.rows(), params.min_time_steps);
  EXPECT_TRUE(result.controls.isZero());
  EXPECT_TRUE(optimizer.controlSequence().vx.isZero());
  EXPECT_TRUE(optimizer.controlSequence().vy.isZero());
  EXPECT_TRUE(optimizer.controlSequence().wz.isZero());
}

TEST(AdaptiveHorizon, RejectsInvalidConfigurationValues)
{
  nav2_mppi_controller::MPPIController controller;
  for (const std::int64_t invalid_minimum : {0, -1}) {
    auto params = makeParams();
    params.min_time_steps = invalid_minimum;
    EXPECT_THROW(controller.configure(params), std::invalid_argument);
  }

  const auto invalid_values = {
    0.0, -1.0, std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid_value : invalid_values) {
    auto params = makeParams();
    params.adaptive_horizon_distance = invalid_value;
    EXPECT_THROW(controller.configure(params), std::invalid_argument);

    params = makeParams();
    params.adaptive_horizon_linear_vel = invalid_value;
    EXPECT_THROW(controller.configure(params), std::invalid_argument);

    params = makeParams();
    params.adaptive_horizon_angular_vel = invalid_value;
    EXPECT_THROW(controller.configure(params), std::invalid_argument);
  }
}

}  // namespace
