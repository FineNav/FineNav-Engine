// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
#define NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_

#include <atomic>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <xtensor/xtensor.hpp>
#include <xtensor/xview.hpp>

#include "finenav_mppi_controller/models/optimizer_settings.hpp"
#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/state.hpp"

namespace mppi
{

/**
 * @class mppi::NoiseGenerator
 * @brief Generates noise trajectories from optimal trajectory.
 *
 * Double-buffered async mode (settings_.regenerate_noises=true):
 *   - Background thread writes into inactive buffer (lock-free).
 *   - Main thread reads from active buffer (lock-free).
 *   - Swap is a single atomic store — ~1 ns on x86, no contention.
 */
class NoiseGenerator
{
public:
  NoiseGenerator() = default;

  void initialize(
    mppi::models::OptimizerSettings & settings,
    bool is_holonomic);

  void shutdown();

  /// Signal background thread to generate next batch into the inactive buffer.
  void generateNextNoises();

  /// Main thread: reads active buffer (no lock) into state controls.
  void setNoisedControls(models::State & state, const models::ControlSequence & control_sequence);

  void reset(mppi::models::OptimizerSettings & settings, bool is_holonomic);

protected:
  void noiseThread();

  /// Generate random controls into the provided buffers (called by background thread).
  void generateNoisedControls(
      xt::xtensor<float, 2> & vx,
      xt::xtensor<float, 2> & vy,
      xt::xtensor<float, 2> & wz);

  /// Returns the active buffer set.
  xt::xtensor<float, 2> & activeVx()  { return active_idx_ ? noises_vx_b_  : noises_vx_a_; }
  xt::xtensor<float, 2> & activeVy()  { return active_idx_ ? noises_vy_b_  : noises_vy_a_; }
  xt::xtensor<float, 2> & activeWz()  { return active_idx_ ? noises_wz_b_  : noises_wz_a_; }

  /// Returns the inactive buffer set (for background thread to fill).
  xt::xtensor<float, 2> & inactiveVx() { return active_idx_ ? noises_vx_a_  : noises_vx_b_; }
  xt::xtensor<float, 2> & inactiveVy() { return active_idx_ ? noises_vy_a_  : noises_vy_b_; }
  xt::xtensor<float, 2> & inactiveWz() { return active_idx_ ? noises_wz_a_  : noises_wz_b_; }

  /// Swap active ↔ inactive (atomic, called by background thread after generation).
  void swapBuffers() { active_idx_ ^= 1; }

  // Double-buffer: A and B
  xt::xtensor<float, 2> noises_vx_a_, noises_vx_b_;
  xt::xtensor<float, 2> noises_vy_a_, noises_vy_b_;
  xt::xtensor<float, 2> noises_wz_a_, noises_wz_b_;

  std::atomic<int> active_idx_{0};  // 0 → A is active, 1 → B is active
  std::atomic<bool> noise_ready_{false};  // background thread sets when done

  mppi::models::OptimizerSettings settings_;
  bool is_holonomic_;

  std::thread noise_thread_;
  std::condition_variable noise_cond_;    // signals background thread
  std::mutex noise_lock_;
  bool active_{false}, ready_{false};
};

}  // namespace mppi

#endif  // NAV2_MPPI_CONTROLLER__TOOLS__NOISE_GENERATOR_HPP_
