// Copyright (c) 2025 Joshua Wallace
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

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <utility>

#include "finenav_route/plugins/graph_file_loaders/geojson_graph_file_loader.hpp"

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
    if (!feature.contains("geometry") || !feature["geometry"].contains("type")) {
      continue;
    }

    if (feature["geometry"]["type"] == "Point") {
      nodes.emplace_back(feature);
    } else if (  // NOLINT
      feature["geometry"]["type"] == "MultiLineString" ||
      feature["geometry"]["type"] == "LineString")
    {
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
    graph[idx].coords = convertCoordinatesFromJson(node);
    graph[idx].operations = convertOperationsFromJson(properties);
    graph[idx].metadata = convertMetaDataFromJson(properties);
    idx++;
  }
}

void GeoJsonGraphFileLoader::addEdgesToGraph(
  Graph & graph, GraphToIDMap & graph_to_id_map, std::vector<Json> & edges)
{
  for (const auto & edge : edges) {
    // Required data
    const auto properties = edge["properties"];
    unsigned int id = properties["id"];
    unsigned int start_id = properties["startid"];
    unsigned int end_id = properties["endid"];

    if (graph_to_id_map.find(start_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "Start id %u does not exist for edge id %u", start_id, id);
      throw finenav_route::NoValidGraph("Start id does not exist");
    }

    if (graph_to_id_map.find(end_id) == graph_to_id_map.end()) {
      RCLCPP_ERROR(logger_, "End id of %u does not exist for edge id %u", end_id, id);
      throw finenav_route::NoValidGraph("End id does not exist");
    }

    EdgeCost edge_cost = convertEdgeCostFromJson(properties);
    Operations operations = convertOperationsFromJson(properties);
    Metadata metadata = convertMetaDataFromJson(properties);
    std::vector<Coordinates> polyline = convertPolylineFromJson(
      edge, graph[graph_to_id_map[start_id]].coords.frame_id);

    graph[graph_to_id_map[start_id]].addEdge(
      edge_cost, &graph[graph_to_id_map[end_id]], id,
      std::move(polyline), metadata, operations);
  }
}

Coordinates GeoJsonGraphFileLoader::convertCoordinatesFromJson(const Json & node)
{
  const auto & properties = node["properties"];
  std::string frame_id{"map"};
  if (properties.contains("frame")) {
    frame_id = properties["frame"];
  }

  const auto & coordinates = node["geometry"]["coordinates"];
  return convertCoordinateTupleFromJson(coordinates, frame_id);
}

Coordinates GeoJsonGraphFileLoader::convertCoordinateTupleFromJson(
  const Json & coordinates, const std::string & frame_id)
{
  if (!coordinates.is_array() || coordinates.size() < 2) {
    throw finenav_route::NoValidGraph("Coordinate tuple must contain at least x and y");
  }

  Coordinates coords;
  coords.frame_id = frame_id;
  coords.x = coordinates[0];
  coords.y = coordinates[1];

  return coords;
}

std::vector<Coordinates> GeoJsonGraphFileLoader::convertPolylineFromJson(
  const Json & edge, const std::string & frame_id)
{
  std::vector<Coordinates> polyline;
  const auto & geometry = edge["geometry"];
  if (!geometry.contains("coordinates")) {
    return polyline;
  }

  const auto & coordinates = geometry["coordinates"];
  if (geometry["type"] == "LineString") {
    polyline.reserve(coordinates.size());
    for (const auto & point : coordinates) {
      polyline.push_back(convertCoordinateTupleFromJson(point, frame_id));
    }
  } else if (geometry["type"] == "MultiLineString") {
    for (const auto & line : coordinates) {
      for (const auto & point : line) {
        polyline.push_back(convertCoordinateTupleFromJson(point, frame_id));
      }
    }
  }

  return polyline;
}

Metadata GeoJsonGraphFileLoader::convertMetaDataFromJson(
  const Json & properties,
  const std::string & key)
{
  Metadata metadata;
  if (!properties.contains(key)) {return metadata;}

  for (const auto & data : properties[key].items()) {
    if (data.value().is_object() ) {
      Metadata new_metadata = convertMetaDataFromJson(properties[key], data.key());
      metadata.setValue(data.key(), new_metadata);
      continue;
    }

    const auto setPrimitiveType = [&](const auto & value) -> std::any
      {
        if (value.is_number()) {
          if (value.is_number_unsigned()) {
            return static_cast<unsigned int>(value);
          } else if (value.is_number_integer()) {
            return static_cast<int>(value);
          } else {
            return static_cast<float>(value);
          }
        }

        if (value.is_boolean()) {
          return static_cast<bool>(value);
        }

        if (value.is_string()) {
          return static_cast<std::string>(value);
        }
        RCLCPP_ERROR(
          logger_, "Failed to convert the key: %s to a value", data.key().c_str());
        throw std::runtime_error("Failed to convert");
      };

    if (data.value().is_array()) {
      std::vector<std::any> array;
      for (const auto & el : data.value()) {
        auto value = setPrimitiveType(el);
        array.push_back(value);
      }
      metadata.setValue(data.key(), array);
      continue;
    }

    auto value = setPrimitiveType(data.value());
    metadata.setValue(data.key(), value);
  }

  return metadata;
}

Operation GeoJsonGraphFileLoader::convertOperationFromJson(const Json & json_operation)
{
  Operation operation;
  json_operation.at("type").get_to(operation.type);
  Json trigger = json_operation.at("trigger");
  operation.trigger = trigger.get<OperationTrigger>();
  Metadata metadata = convertMetaDataFromJson(json_operation);
  operation.metadata = metadata;

  return operation;
}

Operations GeoJsonGraphFileLoader::convertOperationsFromJson(const Json & properties)
{
  Operations operations;
  if (properties.contains("operations")) {
    const auto & json_operations = properties["operations"];
    if (json_operations.is_array()) {
      for (const auto & json_operation : json_operations) {
        operations.push_back(convertOperationFromJson(json_operation));
      }
    } else if (json_operations.is_object()) {
      for (const auto & json_operation : json_operations.items()) {
        operations.push_back(convertOperationFromJson(json_operation.value()));
      }
    }
  }
  return operations;
}

EdgeCost GeoJsonGraphFileLoader::convertEdgeCostFromJson(const Json & properties)
{
  EdgeCost edge_cost;
  if (properties.contains("cost")) {
    edge_cost.cost = properties["cost"];
  }

  if (properties.contains("overridable")) {
    edge_cost.overridable = properties["overridable"];
  }
  return edge_cost;
}

}  // namespace finenav_route
