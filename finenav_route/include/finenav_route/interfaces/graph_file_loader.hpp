// Copyright (c) 2025 Joshua Wallace
// Adapted for FineNav by IWIN-FINS Lab, Shanghai Jiao Tong University.
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

#ifndef FINENAV_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_
#define FINENAV_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_

#include <memory>
#include <string>

#include "finenav_route/types.hpp"

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

#endif  // FINENAV_ROUTE__INTERFACES__GRAPH_FILE_LOADER_HPP_
