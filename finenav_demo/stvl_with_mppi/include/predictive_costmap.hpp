// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// PredictiveCostmap — 从 STVL 动态体素中提取行人轨迹，预测未来位置并注入 costmap。
//
// 最简版本：DBSCAN 聚类 → Kalman 跟踪 → 匀速预测 → 圆形 cost blob。
// 用法：在 TerrainAnalyzer 的 MapInflation 之后，作为 postUpdateHook 调用 update()。

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "grid_map.hpp"
#include "voxel.hpp"
#include "stvl_manager.hpp"
#include "terrain_analysis.hpp"

namespace finenav {

// ============================================================================
// 1. DBSCAN 聚类 — 从动态体素集合中提取独立物体
// ============================================================================
struct ObjectHypothesis {
    Eigen::Vector2d centroid;     // 世界 XY 坐标（所有 cell 的重心）
    std::vector<Position> cells;  // 属于该 cluster 的 cell 位置
    size_t cell_count = 0;
};

inline std::vector<ObjectHypothesis> dbscanCluster(
    const DynamicPosSet & cells,
    double eps      = 0.5,     // 邻域半径 [m]
    size_t min_pts  = 3)       // 最少点数
{
    if (cells.empty()) return {};

    // 复制到 vector 以便索引
    std::vector<Position> points(cells.begin(), cells.end());
    const size_t n = points.size();

    // 邻域查询（暴力 O(n²)，适合动态障碍物数量 < 1000 的场景）
    auto regionQuery = [&](const Position & p) -> std::vector<size_t> {
        std::vector<size_t> neighbors;
        const double eps2 = eps * eps;
        for (size_t i = 0; i < n; ++i) {
            double dx = points[i].x() - p.x();
            double dy = points[i].y() - p.y();
            if (dx * dx + dy * dy <= eps2) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    };

    enum class Label : int8_t { UNVISITED = -1, NOISE = -2 };
    std::vector<Label> labels(n, Label::UNVISITED);
    int cluster_id = 0;

    for (size_t i = 0; i < n; ++i) {
        if (labels[i] != Label::UNVISITED) continue;

        auto neighbors = regionQuery(points[i]);
        if (neighbors.size() < min_pts) {
            labels[i] = Label::NOISE;
            continue;
        }

        // 新 cluster
        Label current_label = static_cast<Label>(cluster_id++);
        labels[i] = current_label;

        // 种子扩展
        std::vector<size_t> seeds = neighbors;
        for (size_t si = 0; si < seeds.size(); ++si) {
            size_t idx = seeds[si];
            if (labels[idx] == Label::NOISE) {
                labels[idx] = current_label;  // 边界点
                continue;
            }
            if (labels[idx] != Label::UNVISITED) continue;

            labels[idx] = current_label;
            auto nbr = regionQuery(points[idx]);
            if (nbr.size() >= min_pts) {
                seeds.insert(seeds.end(), nbr.begin(), nbr.end());
            }
        }
    }

    // 收集结果
    std::vector<ObjectHypothesis> clusters;
    clusters.resize(cluster_id);
    for (auto & c : clusters) {
        c.centroid.setZero();
    }
    std::vector<size_t> counts(cluster_id, 0);

    for (size_t i = 0; i < n; ++i) {
        int cid = static_cast<int>(labels[i]);
        if (cid < 0) continue;  // noise
        clusters[cid].cells.push_back(points[i]);
        clusters[cid].centroid += Eigen::Vector2d(points[i].x(), points[i].y());
        counts[cid]++;
    }

    for (int cid = 0; cid < cluster_id; ++cid) {
        if (counts[cid] > 0) {
            clusters[cid].centroid /= static_cast<double>(counts[cid]);
            clusters[cid].cell_count = counts[cid];
        }
    }

    // 过滤掉空 cluster
    std::vector<ObjectHypothesis> result;
    for (auto & c : clusters) {
        if (c.cell_count >= min_pts) {
            result.push_back(std::move(c));
        }
    }
    return result;
}

// ============================================================================
// 2. Kalman 跟踪器 — 匀速运动模型 (CV)，状态 [px, py, vx, vy]ᵀ
// ============================================================================
struct PersonTrack {
    int id;                          // 全局唯一 track ID
    Eigen::Vector4d state;           // [px, py, vx, vy]
    Eigen::Matrix4d cov;             // 协方差矩阵
    rclcpp::Time last_update;
    rclcpp::Time first_seen;
    size_t hits   = 0;              // 连续命中次数
    size_t misses = 0;              // 连续丢失次数
    bool confirmed = false;

    /// 匀速预测未来 dt 秒后的 2D 位置
    Eigen::Vector2d predictPosition(double dt_sec) const {
        return state.head<2>() + state.tail<2>() * dt_sec;
    }
};

class PeopleTracker {
public:
    /// @param meas_noise_std   观测噪声标准差 [m]（聚类质心不确定性）
    /// @param proc_noise_std   过程噪声标准差 [m/s²]（加速度建模）
    /// @param confirm_hits     确认为有效 track 所需连续命中次数
    /// @param delete_misses    删除 track 前容忍的连续丢失次数
    /// @param max_assoc_dist   关联门限最大距离 [m]
    PeopleTracker(double meas_noise_std   = 0.1,
                  double proc_noise_std   = 0.5,
                  size_t confirm_hits     = 3,
                  size_t delete_misses    = 5,
                  double max_assoc_dist   = 2.0)
        : meas_noise_std_(meas_noise_std)
        , proc_noise_std_(proc_noise_std)
        , confirm_hits_(confirm_hits)
        , delete_misses_(delete_misses)
        , max_assoc_dist_(max_assoc_dist)
    {}

    /// 用一组检测到的 cluster 更新所有 track
    void update(const std::vector<ObjectHypothesis> & detections,
                const rclcpp::Time & now)
    {
        const double dt = (last_update_time_.nanoseconds() > 0)
            ? (now - last_update_time_).seconds()
            : 0.1;
        last_update_time_ = now;

        // ── Step 1: 对所有 track 做一步预测 ──
        for (auto & [id, track] : tracks_) {
            predict(track, dt);
        }

        // ── Step 2: 贪心最近邻关联 ──
        // cost[i][j] = |track_i.predicted - detection_j.centroid|
        std::vector<int> track_ids;
        for (const auto & [id, _] : tracks_) track_ids.push_back(id);

        std::vector<bool> det_matched(detections.size(), false);
        std::vector<bool> trk_matched(track_ids.size(), false);

        // 按距离升序排列所有 (track_i, det_j) 配对
        struct Match { int ti; int dj; double dist; };
        std::vector<Match> candidates;
        for (size_t ti = 0; ti < track_ids.size(); ++ti) {
            const auto & track = tracks_[track_ids[ti]];
            Eigen::Vector2d pred = track.state.head<2>();
            for (size_t dj = 0; dj < detections.size(); ++dj) {
                double dist = (detections[dj].centroid - pred).norm();
                if (dist <= max_assoc_dist_) {
                    candidates.push_back({static_cast<int>(ti), static_cast<int>(dj), dist});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Match & a, const Match & b) { return a.dist < b.dist; });

        for (const auto & m : candidates) {
            if (trk_matched[m.ti] || det_matched[m.dj]) continue;
            trk_matched[m.ti] = true;
            det_matched[m.dj] = true;

            auto & track = tracks_[track_ids[m.ti]];
            update(track, detections[m.dj].centroid);
            track.misses = 0;
            track.hits++;
            if (track.hits >= confirm_hits_) {
                track.confirmed = true;
            }
        }

        // ── Step 3: 未匹配的 track → misses++ ──
        for (size_t ti = 0; ti < track_ids.size(); ++ti) {
            if (!trk_matched[ti]) {
                auto & track = tracks_[track_ids[ti]];
                track.misses++;
                // 仅推进 CV 预测，不更新
            }
        }

        // ── Step 4: 未匹配的 detection → 新 track ──
        for (size_t dj = 0; dj < detections.size(); ++dj) {
            if (det_matched[dj]) continue;
            PersonTrack new_track;
            new_track.id        = next_id_++;
            new_track.state     = Eigen::Vector4d(
                detections[dj].centroid.x(),
                detections[dj].centroid.y(),
                0.0, 0.0);
            new_track.cov       = Eigen::Matrix4d::Identity();
            new_track.cov(0,0)  = 1.0;   // 初始位置不确定性 ~1m
            new_track.cov(1,1)  = 1.0;
            new_track.cov(2,2)  = 2.0;   // 初始速度不确定性 ~1.4 m/s
            new_track.cov(3,3)  = 2.0;
            new_track.last_update = now;
            new_track.first_seen = now;
            new_track.hits = 1;
            tracks_[new_track.id] = new_track;
        }

        // ── Step 5: 清理过期 track ──
        for (auto it = tracks_.begin(); it != tracks_.end(); ) {
            if (it->second.misses > delete_misses_) {
                it = tracks_.erase(it);
            } else {
                ++it;
            }
        }
    }

    /// 获取所有已确认的 track
    std::vector<const PersonTrack *> getConfirmedTracks() const {
        std::vector<const PersonTrack *> result;
        for (const auto & [id, t] : tracks_) {
            if (t.confirmed) result.push_back(&t);
        }
        return result;
    }

private:
    // ── Kalman 常量 ──
    // 状态转移: F = [I₂  dt·I₂;  0  I₂]
    // 观测矩阵: H = [I₂  0]
    void predict(PersonTrack & t, double dt) {
        // x̂⁻ = F x̂
        Eigen::Vector4d x_pred;
        x_pred.head<2>() = t.state.head<2>() + t.state.tail<2>() * dt;
        x_pred.tail<2>() = t.state.tail<2>();

        // P⁻ = F P Fᵀ + Q
        Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
        F(0,2) = dt; F(1,3) = dt;

        // 离散白噪声加速度模型 (DWNA): Q = G q Gᵀ  where G = [dt²/2·I₂, dt·I₂]ᵀ
        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
        const double q = proc_noise_std_ * proc_noise_std_;
        const double dt2 = dt * dt;
        const double dt3_2 = dt2 * dt / 2.0;   // dt³/2
        const double dt4_4 = dt2 * dt2 / 4.0;  // dt⁴/4
        Q(0,0) = q * dt4_4; Q(0,2) = q * dt3_2;
        Q(2,0) = q * dt3_2; Q(2,2) = q * dt2;
        Q(1,1) = q * dt4_4; Q(1,3) = q * dt3_2;
        Q(3,1) = q * dt3_2; Q(3,3) = q * dt2;

        t.state = x_pred;
        t.cov   = F * t.cov * F.transpose() + Q;
    }

    void update(PersonTrack & t, const Eigen::Vector2d & z) {
        // H = [I₂  0₂]
        Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
        H(0,0) = 1.0; H(1,1) = 1.0;

        // R = meas_noise² · I₂
        Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (meas_noise_std_ * meas_noise_std_);

        // y = z - H x̂⁻  (innovation)
        Eigen::Vector2d y = z - H * t.state;

        // S = H P⁻ Hᵀ + R
        Eigen::Matrix2d S = H * t.cov * H.transpose() + R;

        // K = P⁻ Hᵀ S⁻¹
        Eigen::Matrix<double, 4, 2> K = t.cov * H.transpose() * S.inverse();

        // x̂ = x̂⁻ + K y
        t.state += K * y;

        // P = (I₄ - K H) P⁻
        Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
        t.cov = (I4 - K * H) * t.cov;

        t.last_update = last_update_time_;
    }

    std::unordered_map<int, PersonTrack> tracks_;
    int next_id_ = 0;
    rclcpp::Time last_update_time_;

    double meas_noise_std_;
    double proc_noise_std_;
    size_t confirm_hits_;
    size_t delete_misses_;
    double max_assoc_dist_;
};

// ============================================================================
// 3. 轨迹预测 — 匀速外推 + 时间衰减 cost
// ============================================================================
struct PredictedTrajectory {
    int track_id;
    std::vector<Eigen::Vector2d> waypoints;   // 未来各时间步的世界 XY
    double confidence;                         // 0-1
};

inline std::vector<PredictedTrajectory> predictTrajectories(
    const std::vector<const PersonTrack *> & tracks,
    double horizon_sec  = 3.0,
    double dt_sec       = 0.1)
{
    std::vector<PredictedTrajectory> result;
    const int steps = static_cast<int>(horizon_sec / dt_sec);

    for (const auto * track : tracks) {
        PredictedTrajectory traj;
        traj.track_id = track->id;

        // 置信度：基于命中次数和协方差
        double hits_confidence = std::min(1.0, static_cast<double>(track->hits) / 10.0);
        double cov_confidence  = 1.0 / (1.0 + track->cov(0,0) + track->cov(1,1));
        traj.confidence = hits_confidence * cov_confidence;

        for (int i = 1; i <= steps; ++i) {
            double t = i * dt_sec;
            traj.waypoints.push_back(track->predictPosition(t));
        }

        result.push_back(std::move(traj));
    }
    return result;
}

// ============================================================================
// 4. 预测障碍物注入 costmap
// ============================================================================
class PredictiveCostmapPainter {
public:
    /// @param base_radius_m        基础碰撞半径 [m]（≈行人半径）
    /// @param uncertainty_growth   不确定性增长率 [m/s]
    /// @param max_cost             最大代价 (0-254)
    PredictiveCostmapPainter(double base_radius_m        = 0.3,
                             double uncertainty_growth   = 0.1,
                             uint8_t max_cost            = 254)
        : base_radius_m_(base_radius_m)
        , uncertainty_growth_(uncertainty_growth)
        , max_cost_(max_cost)
    {}

    /// 将预测轨迹叠加到 costmap 上
    /// @param costmap      可写的 costmap cv::Mat (CV_8UC1, 0=free, 254=lethal)
    /// @param trajectories 预测轨迹列表
    /// @param now          当前时间
    /// @param resolution   地图分辨率 [m/cell]
    /// @param origin       地图原点在世界坐标系中的左下角位置
    void paint(cv::Mat & costmap,
               const std::vector<PredictedTrajectory> & trajectories,
               const rclcpp::Time & now,
               double resolution,
               const Position & origin)
    {
        const int cols = costmap.cols;
        const int rows = costmap.rows;

        for (const auto & traj : trajectories) {
            for (size_t i = 0; i < traj.waypoints.size(); ++i) {
                // 计算预测时刻相对当前的时间
                double t_sec = (i + 1) * 0.1;  // 默认步长 0.1s

                // cost 随时间衰减: 1/(1+t)
                double cost_factor = 1.0 / (1.0 + t_sec) * traj.confidence;
                if (cost_factor < 0.05) continue;  // 太弱，跳过

                uint8_t cost = static_cast<uint8_t>(std::min(254.0, max_cost_ * cost_factor));

                // 不确定性膨胀半径
                double radius_m = base_radius_m_ + uncertainty_growth_ * t_sec;
                int radius_px = static_cast<int>(std::ceil(radius_m / resolution));

                // 世界坐标 → 像素坐标
                const auto & wp = traj.waypoints[i];
                int cx = static_cast<int>((wp.x() - origin.x()) / resolution);
                int cy = static_cast<int>((wp.y() - origin.y()) / resolution);

                // 画圆
                drawFilledCircle(costmap, cx, cy, radius_px, cost);
            }
        }
    }

private:
    void drawFilledCircle(cv::Mat & img, int cx, int cy, int radius, uint8_t cost) {
        const int r2 = radius * radius;
        const int x0 = std::max(0, cx - radius);
        const int x1 = std::min(img.cols - 1, cx + radius);
        const int y0 = std::max(0, cy - radius);
        const int y1 = std::min(img.rows - 1, cy + radius);

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                int dx = x - cx;
                int dy = y - cy;
                if (dx * dx + dy * dy <= r2) {
                    uint8_t & pixel = img.at<uint8_t>(y, x);
                    pixel = std::max(pixel, cost);  // 取最大值，不覆盖静态障碍物
                }
            }
        }
    }

    double base_radius_m_;
    double uncertainty_growth_;
    uint8_t max_cost_;
};

// ============================================================================
// 5. 顶层 PredictiveCostmap — 一站式调用
// ============================================================================
class PredictiveCostmap {
public:
    PredictiveCostmap(rclcpp::Node::SharedPtr node)
        : node_(std::move(node))
    {
        // 声明可调参数
        node_->declare_parameter<double>("prediction.cluster_eps", 0.5);
        node_->declare_parameter<double>("prediction.cluster_min_pts", 3.0);
        node_->declare_parameter<double>("prediction.meas_noise_std", 0.1);
        node_->declare_parameter<double>("prediction.proc_noise_std", 0.5);
        node_->declare_parameter<double>("prediction.horizon_sec", 3.0);
        node_->declare_parameter<double>("prediction.base_radius", 0.3);
        node_->declare_parameter<double>("prediction.uncertainty_growth", 0.1);
        node_->declare_parameter<int>("prediction.confirm_hits", 3);
        node_->declare_parameter<int>("prediction.delete_misses", 5);
        node_->declare_parameter<double>("prediction.max_assoc_dist", 2.0);
    }

    /// 主更新函数 — 作为 MapServer 的 postUpdateHook 调用（在 TerrainAnalyzer 之后）。
    /// @param stvl_manager     STVL 管理器，提供 dynamic_cells_
    /// @param terrain_analyzer 地形分析器，提供可写的 costmap
    /// @param map              当前 GridMap（用于获取 origin 和分辨率）
    /// @param T_map_body       机器人当前在世界坐标系中的位姿
    void update(StvlManager & stvl_manager,
                TerrainAnalyzer & terrain_analyzer,
                const GridMap<Voxel> & map,
                const Eigen::Isometry3d & T_map_body)
    {
        const rclcpp::Time now = node_->get_clock()->now();

        // 读取动态参数（支持 ros2 param set 在线调参）
        const double cluster_eps         = node_->get_parameter("prediction.cluster_eps").as_double();
        const int    cluster_min_pts     = node_->get_parameter("prediction.cluster_min_pts").as_int();
        const double horizon_sec         = node_->get_parameter("prediction.horizon_sec").as_double();
        const double base_radius         = node_->get_parameter("prediction.base_radius").as_double();
        const double uncertainty_growth  = node_->get_parameter("prediction.uncertainty_growth").as_double();

        // ── Step 1: DBSCAN 聚类 ──
        const auto & dynamic_cells = stvl_manager.getDynamicCells();
        auto detections = dbscanCluster(dynamic_cells, cluster_eps,
                                        static_cast<size_t>(cluster_min_pts));

        // ── Step 2: Kalman 跟踪（tracker 保持跨帧状态）──
        tracker_.update(detections, now);

        // ── Step 3: 轨迹预测 ──
        auto confirmed = tracker_.getConfirmedTracks();
        auto trajectories = predictTrajectories(confirmed, horizon_sec, 0.1);

        // ── Step 4: 注入 costmap ──
        const double resolution = terrain_analyzer.getResolution();
        const Position origin = map.getWindowBounds().min();

        PredictiveCostmapPainter painter(base_radius, uncertainty_growth);
        painter.paint(terrain_analyzer.getMutableCostmap(),
                      trajectories, now, resolution, origin);
    }

private:
    rclcpp::Node::SharedPtr node_;
    PeopleTracker tracker_;  // 跨帧保持状态（track 历史）
};

}  // namespace finenav
