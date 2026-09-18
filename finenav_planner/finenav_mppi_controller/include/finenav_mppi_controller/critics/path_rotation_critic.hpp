// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#ifndef FINENAV_MPPI_CONTROLLER__CRITICS__PATH_ROTATION_CRITIC_HPP_
#define FINENAV_MPPI_CONTROLLER__CRITICS__PATH_ROTATION_CRITIC_HPP_

#include "finenav_mppi_controller/critic_function.hpp"

namespace mppi::critics
{

class PathRotationCritic : public CriticFunction
{
public:
  void initialize(const finenav_mppi_controller::Params & params) override;

  void score(
    CriticData & data,
    const nav2_mppi_controller::IMapView & map_view) override;

protected:
  unsigned int power_{1};
  float weight_{0.0F};
};

}  // namespace mppi::critics

#endif  // FINENAV_MPPI_CONTROLLER__CRITICS__PATH_ROTATION_CRITIC_HPP_
