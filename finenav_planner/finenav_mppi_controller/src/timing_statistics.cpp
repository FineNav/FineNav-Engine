// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_mppi_controller/timing_statistics.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <utility>

namespace nav2_mppi_controller
{

void TimingStatistics::setEnabled(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_ != enabled) {
    samples_.clear();
  }
  enabled_ = enabled;
}

bool TimingStatistics::isEnabled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return enabled_;
}

void TimingStatistics::addSample(const std::string & name, double duration_ms)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (enabled_) {
    samples_[name].push_back(duration_ms);
  }
}

std::map<std::string, TimingSummary> TimingStatistics::summaries() const
{
  std::map<std::string, std::vector<double>> samples;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples = samples_;
  }

  std::map<std::string, TimingSummary> result;
  for (auto & [name, values] : samples) {
    if (values.empty()) {
      continue;
    }

    std::sort(values.begin(), values.end());
    const auto count = values.size();
    const double median = count % 2 == 0 ?
      (values[count / 2 - 1] + values[count / 2]) / 2.0 :
      values[count / 2];

    result.emplace(
      name, TimingSummary{
        count,
        values.front(),
        values.back(),
        std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(count),
        median});
  }
  return result;
}

bool TimingStatistics::writeReport(
  const std::filesystem::path & path, std::string & error) const
{
  if (!isEnabled()) {
    error = "timing statistics are disabled";
    return false;
  }

  const auto report = summaries();
  std::ofstream output(path);
  if (!output) {
    error = "failed to open " + path.string();
    return false;
  }

  const auto plan = report.find("CompletePlan");
  const std::size_t total_planning_frames =
    plan == report.end() ? 0 : plan->second.count;

  output << "MPPI timing statistics (milliseconds)\n";
  output << "total_planning_frames: " << total_planning_frames << "\n\n";
  output << "item\tcount\tmin_ms\tmax_ms\taverage_ms\tmedian_ms\n";
  output << std::fixed << std::setprecision(3);
  for (const auto & [name, summary] : report) {
    output << name << '\t'
           << summary.count << '\t'
           << summary.minimum_ms << '\t'
           << summary.maximum_ms << '\t'
           << summary.average_ms << '\t'
           << summary.median_ms << '\n';
  }

  if (!output) {
    error = "failed to write " + path.string();
    return false;
  }
  return true;
}

TimingStatistics & timingStatistics()
{
  static TimingStatistics statistics;
  return statistics;
}

}  // namespace nav2_mppi_controller
