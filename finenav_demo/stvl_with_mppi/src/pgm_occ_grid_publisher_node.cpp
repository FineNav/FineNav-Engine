// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace {

bool readNextToken(std::istream & in, std::string & token)
{
  token.clear();
  while (in >> token) {
    if (!token.empty() && token[0] == '#') {
      std::string dummy;
      std::getline(in, dummy);
      continue;
    }
    return true;
  }
  return false;
}

bool loadPgm(const std::string & path, int & width, int & height, int & maxval, std::vector<uint8_t> & pixels)
{
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return false;
  }

  std::string magic;
  if (!readNextToken(in, magic) || (magic != "P5" && magic != "P2")) {
    return false;
  }

  std::string token;
  if (!readNextToken(in, token)) return false;
  width = std::stoi(token);
  if (!readNextToken(in, token)) return false;
  height = std::stoi(token);
  if (!readNextToken(in, token)) return false;
  maxval = std::stoi(token);

  if (width <= 0 || height <= 0 || maxval <= 0 || maxval > 255) {
    return false;
  }

  pixels.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);

  if (magic == "P2") {
    for (int i = 0; i < width * height; ++i) {
      if (!readNextToken(in, token)) return false;
      int v = std::stoi(token);
      v = std::clamp(v, 0, maxval);
      pixels[static_cast<size_t>(i)] = static_cast<uint8_t>((255 * v) / maxval);
    }
  } else {
    in >> std::ws;
    in.read(reinterpret_cast<char *>(pixels.data()), static_cast<std::streamsize>(pixels.size()));
    if (in.gcount() != static_cast<std::streamsize>(pixels.size())) {
      return false;
    }
    if (maxval != 255) {
      for (auto & p : pixels) {
        p = static_cast<uint8_t>((255 * p) / maxval);
      }
    }
  }

  return true;
}

}  // namespace

class PgmOccGridPublisherNode : public rclcpp::Node
{
public:
  PgmOccGridPublisherNode()
  : Node("pgm_occ_grid_publisher_node")
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    pgm_path_ = declare_parameter<std::string>(
      "pgm_path",
      "flat_small_scale_maze_2d.pgm");

    resolution_ = declare_parameter<double>("resolution", 0.2);
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    origin_yaw_ = declare_parameter<double>("origin_yaw", 0.0);

    occupied_thresh_ = declare_parameter<int>("occupied_thresh", 80);
    free_thresh_ = declare_parameter<int>("free_thresh", 20);
    publish_hz_ = declare_parameter<double>("publish_hz", 1.0);

    occupied_thresh_ = std::clamp(occupied_thresh_, 0, 100);
    free_thresh_ = std::clamp(free_thresh_, 0, 100);

    pgm_path_ = resolvePgmPath(pgm_path_);

    if (!buildMapMsg()) {
      std::ostringstream oss;
      oss << "failed to load pgm map: " << pgm_path_;
      if (!path_candidates_.empty()) {
        oss << " | candidates:";
        for (const auto & c : path_candidates_) {
          oss << "\n  - " << c;
        }
      }
      throw std::runtime_error(oss.str());
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, qos);

    const auto period = std::chrono::duration<double>(1.0 / std::max(publish_hz_, 0.1));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PgmOccGridPublisherNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "PGM map loaded: %s (%u x %u, res=%.3f), publish topic=%s",
      pgm_path_.c_str(), map_msg_.info.width, map_msg_.info.height, resolution_, map_topic_.c_str());
  }

private:
  std::string resolvePgmPath(const std::string & raw_path)
  {
    namespace fs = std::filesystem;
    path_candidates_.clear();

    auto try_path = [&](const fs::path & p) -> std::string {
      path_candidates_.push_back(p.string());
      if (fs::exists(p) && fs::is_regular_file(p)) {
        return p.string();
      }
      return std::string{};
    };

    fs::path raw(raw_path);
    if (raw.is_absolute()) {
      auto ok = try_path(raw);
      return ok.empty() ? raw_path : ok;
    }

    if (auto ok = try_path(fs::current_path() / raw); !ok.empty()) {
      return ok;
    }

    try {
      const auto share = fs::path(ament_index_cpp::get_package_share_directory("finenav_demo"));
      if (auto ok = try_path(share / "config" / raw); !ok.empty()) {
        return ok;
      }
      if (auto ok = try_path(share / raw); !ok.empty()) {
        return ok;
      }
    } catch (const std::exception &) {
      // Keep raw path; detailed diagnostics are reported via path_candidates_.
    }

    return raw_path;
  }

  bool buildMapMsg()
  {
    int width = 0;
    int height = 0;
    int maxval = 0;
    std::vector<uint8_t> pixels;

    if (!loadPgm(pgm_path_, width, height, maxval, pixels)) {
      return false;
    }

    map_msg_ = nav_msgs::msg::OccupancyGrid{};
    map_msg_.info.width = static_cast<uint32_t>(width);
    map_msg_.info.height = static_cast<uint32_t>(height);
    map_msg_.info.resolution = static_cast<float>(resolution_);
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.position.z = 0.0;

    const double half = 0.5 * origin_yaw_;
    map_msg_.info.origin.orientation.x = 0.0;
    map_msg_.info.origin.orientation.y = 0.0;
    map_msg_.info.origin.orientation.z = static_cast<double>(std::sin(static_cast<long double>(half)));
    map_msg_.info.origin.orientation.w = static_cast<double>(std::cos(static_cast<long double>(half)));

    map_msg_.data.assign(static_cast<size_t>(width) * static_cast<size_t>(height), 0);

    for (int gy = 0; gy < height; ++gy) {
      for (int gx = 0; gx < width; ++gx) {
        const int img_y = height - 1 - gy;  // image top-left -> map bottom-left
        const size_t img_idx = static_cast<size_t>(img_y) * static_cast<size_t>(width) + static_cast<size_t>(gx);
        const uint8_t gray = pixels[img_idx];

        // 0=black(occupied), 255=white(free)
        const int occ_pct = static_cast<int>(std::lround((255.0 - static_cast<double>(gray)) * 100.0 / 255.0));

        int8_t val = -1;
        if (occ_pct >= occupied_thresh_) {
          val = 100;
        } else if (occ_pct <= free_thresh_) {
          val = 0;
        }

        const size_t map_idx = static_cast<size_t>(gy) * static_cast<size_t>(width) + static_cast<size_t>(gx);
        map_msg_.data[map_idx] = val;
      }
    }

    return true;
  }

  void onTimer()
  {
    map_msg_.header.stamp = now();
    map_msg_.header.frame_id = map_frame_;
    map_pub_->publish(map_msg_);
  }

private:
  std::string map_topic_;
  std::string map_frame_;
  std::string pgm_path_;
  std::vector<std::string> path_candidates_;

  double resolution_{0.2};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double publish_hz_{1.0};

  int occupied_thresh_{80};
  int free_thresh_{20};

  nav_msgs::msg::OccupancyGrid map_msg_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PgmOccGridPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

