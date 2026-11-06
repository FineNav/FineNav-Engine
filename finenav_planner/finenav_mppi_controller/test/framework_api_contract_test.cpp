// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <concepts>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "finenav_core/concepts.hpp"
#include "finenav_mppi_controller/controller.hpp"
#include "finenav_mppi_controller/timing_statistics.hpp"

namespace
{

using Controller = nav2_mppi_controller::MPPIController;

static_assert(finenav::PlannerConcept<Controller>);
static_assert(std::same_as<Controller::ConfigType, finenav_mppi_controller::Params>);
static_assert(std::same_as<Controller::Context, finenav::PlanningContext<Controller>>);
static_assert(std::same_as<Controller::IMapView, nav2_mppi_controller::IMapView>);
static_assert(
  std::same_as<
    decltype(std::declval<Controller &>().plan(
      std::declval<const Controller::Context &>(),
      std::declval<const Controller::IMapView &>())),
    std::optional<finenav::OutputDataT<Controller>>>);

static_assert(Controller::InputProfile.pose);
static_assert(!Controller::InputProfile.vel);
static_assert(!Controller::InputProfile.accel);
static_assert(!Controller::InputProfile.time);
static_assert(Controller::OutputProfile.pose);
static_assert(Controller::OutputProfile.vel);
static_assert(!Controller::OutputProfile.accel);
static_assert(!Controller::OutputProfile.time);

class FrameworkMapView final : public Controller::IMapView
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

TEST(MPPIFrameworkAPIContract, EmptyReferenceReturnsNoPlan)
{
  Controller controller;
  Controller::Context context;
  FrameworkMapView map_view;

  EXPECT_FALSE(controller.plan(context, map_view).has_value());
}

TEST(MPPIFrameworkAPIContract, ConfigureAppliesTimingSetting)
{
  nav2_mppi_controller::timingStatistics().setEnabled(false);

  Controller enabled_controller;
  Controller::ConfigType enabled_params;
  enabled_params.timing_statistics_enabled = true;
  enabled_controller.configure(enabled_params);
  EXPECT_TRUE(nav2_mppi_controller::timingStatistics().isEnabled());

  Controller disabled_controller;
  Controller::ConfigType disabled_params;
  disabled_controller.configure(disabled_params);
  EXPECT_FALSE(nav2_mppi_controller::timingStatistics().isEnabled());
}

TEST(MPPIFrameworkAPIContract, DisjointFeedbackVelocityReturnsZeroControlSequence)
{
  FrameworkMapView map_view;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.orientation.w = 1.0;
  nav_msgs::msg::Path path;

  for (const auto & [motion_model, vx, vy, wz] : {
    std::tuple{"DiffDrive", 0.58, 0.0, 0.0},
    std::tuple{"Omni", 0.0, 0.58, 0.0},
    std::tuple{"DiffDrive", 0.0, 0.0, 0.58}}) {
    finenav_mppi_controller::Params params;
    params.motion_model = motion_model;
    params.vx_min = -0.4;
    params.vx_max = 0.4;
    params.vy_max = 0.4;
    params.wz_max = 0.4;
    params.ax_min = -3.0;
    params.ax_max = 3.0;
    params.ay_min = -3.0;
    params.ay_max = 3.0;
    params.az_max = 3.0;
    params.time_steps = 3;
    params.batch_size = 2;
    params.regenerate_noises = false;

    mppi::Optimizer optimizer;
    std::vector<std::unique_ptr<mppi::critics::CriticFunction>> critics;
    optimizer.initialize("test", critics, params);
    geometry_msgs::msg::Twist speed;
    speed.linear.x = vx;
    speed.linear.y = vy;
    speed.angular.z = wz;

    const auto result = optimizer.evalControl(pose, speed, path, map_view);
    EXPECT_EQ(result.controls.rows(), 3);
    EXPECT_TRUE(result.controls.isZero());
    for (Eigen::Index i = 0; i < result.trajectory.rows(); ++i) {
      EXPECT_FLOAT_EQ(result.trajectory(i, 0), 1.0F);
      EXPECT_FLOAT_EQ(result.trajectory(i, 1), 2.0F);
      EXPECT_FLOAT_EQ(result.trajectory(i, 2), 0.0F);
    }
  }
}

}  // namespace
