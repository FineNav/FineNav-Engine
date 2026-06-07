// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "route/route_exceptions.hpp"
#include "route/interfaces/graph_file_loader.hpp"

namespace finenav_route
{

class GeoJsonGraphFileLoader : public GraphFileLoader
{
public:
  using Json = nlohmann::json;

  GeoJsonGraphFileLoader() = default;
  ~GeoJsonGraphFileLoader() = default;

  bool loadGraphFromFile(
    Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath) override;

  bool loadGraphFromJson(
    Graph & graph, GraphToIDMap & graph_to_id_map, const std::string & graph_json);

protected:
  bool doesFileExist(const std::string & filepath);

  void getGraphElements(
    const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges);

  void addNodesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes);

  void addEdgesToGraph(Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges);

  Coordinates convertCoordinatesFromJson(const Json & node);

  Coordinates convertCoordinateTupleFromJson(
    const Json & coordinates, const std::string & frame_id = "map");

  std::vector<Coordinates> convertPolylineFromJson(
    const Json & edge, const std::string & frame_id = "map");

  Metadata convertMetaDataFromJson(const Json & properties, const std::string & key = "metadata");

  Operation convertOperationFromJson(const Json & json_operation);

  Operations convertOperationsFromJson(const Json & properties);

  EdgeCost convertEdgeCostFromJson(const Json & properties);

  rclcpp::Logger logger_{rclcpp::get_logger("GeoJsonGraphFileLoader")};
};

NLOHMANN_JSON_SERIALIZE_ENUM(
  OperationTrigger, {
    {OperationTrigger::NODE,     "NODE"},
    {OperationTrigger::ON_ENTER, "ON_ENTER"},
    {OperationTrigger::ON_EXIT,  "ON_EXIT"},
  })

}  // namespace finenav_route

