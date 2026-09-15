// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace nav2_mppi_controller
{

struct TimingSummary
{
  std::size_t count{0};
  double minimum_ms{0.0};
  double maximum_ms{0.0};
  double average_ms{0.0};
  double median_ms{0.0};
};

class TimingStatistics
{
public:
  void setEnabled(bool enabled);
  bool isEnabled() const;
  void addSample(const std::string & name, double duration_ms);
  std::map<std::string, TimingSummary> summaries() const;
  bool writeReport(const std::filesystem::path & path, std::string & error) const;

private:
  mutable std::mutex mutex_;
  bool enabled_{false};
  std::map<std::string, std::vector<double>> samples_;
};

TimingStatistics & timingStatistics();

}  // namespace nav2_mppi_controller
