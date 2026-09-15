// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_mppi_controller/timing_statistics.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace
{

using nav2_mppi_controller::TimingStatistics;

TEST(TimingStatisticsTest, ComputesSummaryForOddAndEvenSampleCounts)
{
  TimingStatistics statistics;
  statistics.setEnabled(true);

  statistics.addSample("CompletePlan", 9.0);
  statistics.addSample("CompletePlan", 1.0);
  statistics.addSample("CompletePlan", 5.0);
  statistics.addSample("CompletePlan", 3.0);
  statistics.addSample("GoalCritic", 30.0);
  statistics.addSample("GoalCritic", 10.0);
  statistics.addSample("GoalCritic", 20.0);

  const auto summaries = statistics.summaries();
  const auto & plan = summaries.at("CompletePlan");
  EXPECT_EQ(plan.count, 4U);
  EXPECT_DOUBLE_EQ(plan.minimum_ms, 1.0);
  EXPECT_DOUBLE_EQ(plan.maximum_ms, 9.0);
  EXPECT_DOUBLE_EQ(plan.average_ms, 4.5);
  EXPECT_DOUBLE_EQ(plan.median_ms, 4.0);

  const auto & goal = summaries.at("GoalCritic");
  EXPECT_EQ(goal.count, 3U);
  EXPECT_DOUBLE_EQ(goal.minimum_ms, 10.0);
  EXPECT_DOUBLE_EQ(goal.maximum_ms, 30.0);
  EXPECT_DOUBLE_EQ(goal.average_ms, 20.0);
  EXPECT_DOUBLE_EQ(goal.median_ms, 20.0);
}

TEST(TimingStatisticsTest, IgnoresSamplesWhileDisabled)
{
  TimingStatistics statistics;

  statistics.addSample("CompletePlan", 12.0);

  EXPECT_TRUE(statistics.summaries().empty());
}

TEST(TimingStatisticsTest, StartsFreshSessionWhenTheSwitchIsToggled)
{
  TimingStatistics statistics;
  statistics.setEnabled(true);
  statistics.addSample("CompletePlan", 12.0);

  statistics.setEnabled(false);
  statistics.setEnabled(true);

  EXPECT_TRUE(statistics.summaries().empty());
}

TEST(TimingStatisticsTest, WritesFrameCountAndAllSummaries)
{
  TimingStatistics statistics;
  statistics.setEnabled(true);
  statistics.addSample("CompletePlan", 10.0);
  statistics.addSample("CompletePlan", 20.0);
  statistics.addSample("CostCritic", 2.0);

  const auto report_path =
    std::filesystem::temp_directory_path() / "finenav_mppi_timing_statistics_test.txt";
  std::filesystem::remove(report_path);

  std::string error;
  ASSERT_TRUE(statistics.writeReport(report_path, error)) << error;

  std::ifstream report(report_path);
  const std::string contents{
    std::istreambuf_iterator<char>(report), std::istreambuf_iterator<char>()};
  EXPECT_NE(contents.find("total_planning_frames: 2"), std::string::npos);
  EXPECT_NE(
    contents.find("CompletePlan\t2\t10.000\t20.000\t15.000\t15.000"),
    std::string::npos);
  EXPECT_NE(
    contents.find("CostCritic\t1\t2.000\t2.000\t2.000\t2.000"),
    std::string::npos);

  std::filesystem::remove(report_path);
}

}  // namespace
