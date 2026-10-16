// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_common/geometry_types.hpp"

namespace finenav::example {

class DummyWorldView {
  public:
    virtual ~DummyWorldView() = default;
    virtual bool isWalkable(const finenav::common::Position3D& position) const = 0;
};

}  // namespace finenav::example
