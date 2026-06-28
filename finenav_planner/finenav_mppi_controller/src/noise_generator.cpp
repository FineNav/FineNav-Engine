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

#include "finenav_mppi_controller/tools/noise_generator.hpp"

#include <memory>
#include <mutex>
#include <xtensor/xmath.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xnoalias.hpp>

namespace mppi
{

void NoiseGenerator::initialize(
  mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;
  active_ = true;

  // Allocate both buffers
  auto sz = std::vector<std::size_t>{settings_.batch_size, settings_.time_steps};
  noises_vx_a_.resize(sz);  noises_vx_b_.resize(sz);
  noises_wz_a_.resize(sz);  noises_wz_b_.resize(sz);
  if (is_holonomic_) {
    noises_vy_a_.resize(sz);  noises_vy_b_.resize(sz);
  }

  if (regenerate_noises_) {
    // Generate first batch into active buffer synchronously,
    // then start background thread for subsequent batches.
    generateNoisedControls(activeVx(), activeVy(), activeWz());
    noise_ready_.store(true, std::memory_order_release);
    noise_thread_ = std::thread(std::bind(&NoiseGenerator::noiseThread, this));
  } else {
    generateNoisedControls(activeVx(), activeVy(), activeWz());
  }
}

void NoiseGenerator::shutdown()
{
  active_ = false;
  ready_ = true;
  noise_cond_.notify_all();
  if (noise_thread_.joinable()) {
    noise_thread_.join();
  }
}

void NoiseGenerator::generateNextNoises()
{
  noise_ready_.store(false, std::memory_order_release);
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    ready_ = true;
  }
  noise_cond_.notify_all();
}

void NoiseGenerator::setNoisedControls(
  models::State & state,
  const models::ControlSequence & control_sequence)
{
  if (regenerate_noises_) {
    // Wait for background thread — common case: already ready, worst case: ~noise_gen_time.
    while (!noise_ready_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    noise_ready_.store(false, std::memory_order_release);
  }

  auto & vx = activeVx();
  auto & vy = activeVy();
  auto & wz = activeWz();

  xt::noalias(state.cvx) = control_sequence.vx + vx;
  xt::noalias(state.cvy) = control_sequence.vy + vy;
  xt::noalias(state.cwz) = control_sequence.wz + wz;
}

void NoiseGenerator::reset(mppi::models::OptimizerSettings & settings, bool is_holonomic)
{
  settings_ = settings;
  is_holonomic_ = is_holonomic;

  // Zero both buffers, signal thread to regenerate
  {
    std::unique_lock<std::mutex> guard(noise_lock_);
    xt::noalias(activeVx())   = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(activeVy())   = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(activeWz())   = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(inactiveVx()) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(inactiveVy()) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    xt::noalias(inactiveWz()) = xt::zeros<float>({settings_.batch_size, settings_.time_steps});
    ready_ = true;
  }

  if (regenerate_noises_) {
    noise_cond_.notify_all();
  } else {
    generateNoisedControls(activeVx(), activeVy(), activeWz());
  }
}

void NoiseGenerator::noiseThread()
{
  do {
    {
      std::unique_lock<std::mutex> guard(noise_lock_);
      noise_cond_.wait(guard, [this]() { return ready_; });
      ready_ = false;
    }
    // Generate into inactive buffer — no lock held during ~6 ms randn
    generateNoisedControls(inactiveVx(), inactiveVy(), inactiveWz());
    // Atomic swap: active ↔ inactive (no lock needed)
    swapBuffers();
    // Signal main thread: fresh noise is ready
    noise_ready_.store(true, std::memory_order_release);
  } while (active_);
}

void NoiseGenerator::generateNoisedControls(
    xt::xtensor<float, 2> & vx,
    xt::xtensor<float, 2> & vy,
    xt::xtensor<float, 2> & wz)
{
  auto & s = settings_;

  xt::noalias(vx) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.vx);
  xt::noalias(wz) = xt::random::randn<float>(
    {s.batch_size, s.time_steps}, 0.0f,
    s.sampling_std.wz);
  if (is_holonomic_) {
    xt::noalias(vy) = xt::random::randn<float>(
      {s.batch_size, s.time_steps}, 0.0f,
      s.sampling_std.vy);
  }
}

}  // namespace mppi
