// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"

namespace finenav_route
{

bool GeoJsonGraphFileLoader::loadGraphFromFile(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::string filepath)
{
  if (!doesFileExist(filepath)) {
    RCLCPP_ERROR(logger_, "The filepath %s does not exist", filepath.c_str());
    return false;
  }
  std::ifstream graph_file(filepath);
  Json json_graph;
  try {
    json_graph = Json::parse(graph_file);
  } catch (Json::parse_error & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse %s: %s", filepath.c_str(), ex.what());
    return false;
  }
  return loadGraphFromJson(graph, graph_to_id_map, json_graph.dump());
}

bool GeoJsonGraphFileLoader::loadGraphFromJson(
  Graph & graph, GraphToIDMap & graph_to_id_map, const std::string & graph_json)
{
  Json json_graph;
  try {
    json_graph = Json::parse(graph_json);
  } catch (Json::parse_error & ex) {
    RCLCPP_ERROR(logger_, "Failed to parse route graph json: %s", ex.what());
    return false;
  }
  if (!json_graph.contains("features") || !json_graph["features"].is_array()) {
    RCLCPP_ERROR(logger_, "The route graph json does not contain a features array");
    return false;
  }

  auto features = json_graph["features"];
  std::vector<Json> nodes, edges;
  getGraphElements(features, nodes, edges);

  if (nodes.empty() || edges.empty()) {
    RCLCPP_ERROR(logger_, "The route graph json is malformed. It does not contain nodes or edges.");
    return false;
  }

  graph.clear();
  graph_to_id_map.clear();
  graph.resize(nodes.size());
  addNodesToGraph(graph, graph_to_id_map, nodes);
  addEdgesToGraph(graph, graph_to_id_map, edges);
  return true;
}

bool GeoJsonGraphFileLoader::doesFileExist(const std::string & filepath)
{
  return std::filesystem::exists(filepath);
}

void GeoJsonGraphFileLoader::getGraphElements(
  const Json & features, std::vector<Json> & nodes, std::vector<Json> & edges)
{
  for (const auto & feature : features) {
    if (!feature.contains("geometry") || !feature["geometry"].contains("type")) continue;
    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    } else if (feature["geometry"]["type"] == "MultiLineString" ||
               feature["geometry"]["type"] == "LineString") {
      edges.emplace_back(feature);
    }
  }
}

void GeoJsonGraphFileLoader::addNodesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & nodes)
{
  int idx = 0;
  for (const auto & node : nodes) {
    const auto properties = node["properties"];
    graph[idx].nodeid = properties["id"];
    graph_to_id_map[graph[idx].nodeid] = idx;
    graph[idx].coords     = convertCoordinatesFromJson(node);
    graph[idx].operations = convertOperationsFromJson(properties);
    graph[idx].metadata   = convertMetaDataFromJson(properties);
    idx++;
  }
}

void GeoJsonGraphFileLoader::addEdgesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges)
{
  for (const auto & edge : edges) {
    const auto properties = edge["properties"];
    unsigned int id       = properties["id"];
    unsigned int start_id = properties["startid"];
    unsigned int end_id   = properties["endid"];

    if (!graph_to_id_map.count(start_id)) {
      RCLCPP_ERROR(logger_, "Start id %u does not exist for edge id %u", start_id, id);
      throw finenav_route::NoValidGraph("Start id does not exist");
    }
    if (!graph_to_id_map.count(end_id)) {
      RCLCPP_ERROR(logger_, "End id of %u does not exist for edge id %u", end_id, id);
      throw finenav_route::NoValidGraph("End id does not exist");
    }

    EdgeCost edge_cost           = convertEdgeCostFromJson(properties);
    Operations operations        = convertOperationsFromJson(properties);
    Metadata metadata            = convertMetaDataFromJson(properties);
    std::vector<Coordinates> pl  = convertPolylineFromJson(
      edge, graph[graph_to_id_map[start_id]].coords.frame_id);

    graph[graph_to_id_map[start_id]].addEdge(
      edge_cost, &graph[graph_to_id_map[end_id]], id, std::move(pl), metadata, operations);
  }
}

Coordinates GeoJsonGraphFileLoader::convertCoordinatesFromJson(const Json & node)
{
  const auto & properties = node["properties"];
  std::string frame_id{"map"};
  if (properties.contains("frame")) frame_id = properties["frame"];
  return convertCoordinateTupleFromJson(node["geometry"]["coordinates"], frame_id);
}

Coordinates GeoJsonGraphFileLoader::convertCoordinateTupleFromJson(
  const Json & coordinates, const std::string & frame_id)
{
  if (!coordinates.is_array() || coordinates.size() < 2)
    throw finenav_route::NoValidGraph("Coordinate tuple must contain at least x and y");
  return {frame_id, static_cast<float>(coordinates[0]), static_cast<float>(coordinates[1])};
}

std::vector<Coordinates> GeoJsonGraphFileLoader::convertPolylineFromJson(
  const Json & edge, const std::string & frame_id)
{
  std::vector<Coordinates> polyline;
  const auto & geometry = edge["geometry"];
  if (!geometry.contains("coordinates")) return polyline;
  const auto & coordinates = geometry["coordinates"];
  if (geometry["type"] == "LineString") {
    polyline.reserve(coordinates.size());
    for (const auto & point : coordinates)
      polyline.push_back(convertCoordinateTupleFromJson(point, frame_id));
  } else if (geometry["type"] == "MultiLineString") {
    for (const auto & line : coordinates)
      for (const auto & point : line)
        polyline.push_back(convertCoordinateTupleFromJson(point, frame_id));
  }
  return polyline;
}

Metadata GeoJsonGraphFileLoader::convertMetaDataFromJson(
  const Json & properties, const std::string & key)
{
  Metadata metadata;
  if (!properties.contains(key)) return metadata;
  for (const auto & data : properties[key].items()) {
    if (data.value().is_object()) {
      Metadata sub = convertMetaDataFromJson(properties[key], data.key());
      metadata.setValue(data.key(), sub);
      continue;
    }
    const auto setPrimitive = [&](const auto & value) -> std::any {
      if (value.is_number()) {
        if (value.is_number_unsigned()) return static_cast<unsigned int>(value);
        if (value.is_number_integer())  return static_cast<int>(value);
        return static_cast<float>(value);
      }
      if (value.is_boolean()) return static_cast<bool>(value);
      if (value.is_string())  return static_cast<std::string>(value);
      RCLCPP_ERROR(logger_, "Failed to convert key: %s", data.key().c_str());
      throw std::runtime_error("Failed to convert");
    };
    if (data.value().is_array()) {
      std::vector<std::any> arr;
      for (const auto & el : data.value()) arr.push_back(setPrimitive(el));
      metadata.setValue(data.key(), arr);
      continue;
    }
    auto val = setPrimitive(data.value());
    metadata.setValue(data.key(), val);
  }
  return metadata;
}

Operation GeoJsonGraphFileLoader::convertOperationFromJson(const Json & json_operation)
{
  Operation operation;
  json_operation.at("type").get_to(operation.type);
  operation.trigger = json_operation.at("trigger").get<OperationTrigger>();
  operation.metadata = convertMetaDataFromJson(json_operation);
  return operation;
}

Operations GeoJsonGraphFileLoader::convertOperationsFromJson(const Json & properties)
{
  Operations operations;
  if (!properties.contains("operations")) return operations;
  const auto & json_operations = properties["operations"];
  if (json_operations.is_array()) {
    for (const auto & op : json_operations)
      operations.push_back(convertOperationFromJson(op));
  } else if (json_operations.is_object()) {
    for (const auto & op : json_operations.items())
      operations.push_back(convertOperationFromJson(op.value()));
  }
  return operations;
}

EdgeCost GeoJsonGraphFileLoader::convertEdgeCostFromJson(const Json & properties)
{
  EdgeCost edge_cost;
  if (properties.contains("cost"))       edge_cost.cost       = properties["cost"];
  if (properties.contains("overridable")) edge_cost.overridable = properties["overridable"];
  return edge_cost;
}

}  // namespace finenav_route

