// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>

#include "route/types.hpp"

namespace finenav_route
{

class GraphFileLoader
{
public:
  using Ptr = std::shared_ptr<GraphFileLoader>;

  GraphFileLoader() = default;
  virtual ~GraphFileLoader() = default;

  virtual bool loadGraphFromFile(
    Graph & graph,
    GraphToIDMap & graph_to_id_map,
    std::string filepath) = 0;
};

}  // namespace finenav_route

