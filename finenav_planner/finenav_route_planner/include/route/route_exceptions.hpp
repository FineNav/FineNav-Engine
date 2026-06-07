// Copyright (c) 2025-2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <stdexcept>
#include <string>

namespace finenav_route
{

class RouteException : public std::runtime_error {
public:
  explicit RouteException(const std::string & desc) : std::runtime_error(desc) {}
};

class NoValidRouteCouldBeFound : public RouteException {
public:
  explicit NoValidRouteCouldBeFound(const std::string & desc)
  : RouteException("No valid route could be found: " + desc) {}
};

class TimedOut : public RouteException {
public:
  explicit TimedOut(const std::string & desc)
  : RouteException("Timed out: " + desc) {}
};

class RouteTFError : public RouteException {
public:
  explicit RouteTFError(const std::string & desc)
  : RouteException("Route TF error: " + desc) {}
};

class NoValidGraph : public RouteException {
public:
  explicit NoValidGraph(const std::string & desc)
  : RouteException("No valid graph: " + desc) {}
};

class IndeterminantNodesOnGraph : public RouteException {
public:
  explicit IndeterminantNodesOnGraph(const std::string & desc)
  : RouteException("Indeterminant nodes on graph: " + desc) {}
};

class InvalidEdgeScorerUse : public RouteException {
public:
  explicit InvalidEdgeScorerUse(const std::string & desc)
  : RouteException("Invalid edge scorer use: " + desc) {}
};

class OperationFailed : public RouteException {
public:
  explicit OperationFailed(const std::string & desc)
  : RouteException("Operation failed: " + desc) {}
};

}  // namespace finenav_route

