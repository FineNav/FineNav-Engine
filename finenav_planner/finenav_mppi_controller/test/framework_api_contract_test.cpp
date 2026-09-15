// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <concepts>
#include <optional>
#include <type_traits>
#include <utility>

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

}  // namespace
