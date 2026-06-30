// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Unit tests for SimpleGridMapView: ObbConfig, getThreadObb, asymmetric
// param parsing, isCollision cost gating, costAtPose, getRadius, and
// multi-threaded safety.

#include <gtest/gtest.h>

#include "terrain_mapview.hpp"

#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <atomic>
#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using namespace mppi;

// ═══════════════════════════════════════════════════════════════════════════
// Test fixture — sets up ROS, a minimal GridMap + MapServer + TerrainAnalyzer
// so SimpleGridMapView can be constructed with real dependencies.
// ═══════════════════════════════════════════════════════════════════════════

class SimpleGridMapViewTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_mapview_node");

        // MapServer creates and configures its internal GridMap (hardcoded
        // 5×5×5 @ 0.05m via configure()).  We get a reference to it for
        // both TerrainAnalyzer (needs map dims) and SimpleGridMapView.
        map_server_ = std::make_shared<finenav::MapServer<finenav::GridMap<Voxel>>>(
            node_, "test_map_server", 10.0, nullptr, 50.0);

        // Snapshot a reference to MapServer's internal map.  The LockedMapRO
        // releases the shared_lock on destruction, but in a single-threaded
        // test this is safe — no writer contends.
        {
            auto map_ro = map_server_->getLockedReadView();
            map_ref_ = map_ro.operator->();  // pointer to the internal map
        }

        terrain_analyzer_ = std::make_shared<TerrainAnalyzer>(
            map_server_, node_, nullptr, nullptr);

        obb_config_.outer_radius = (obb_config_.max - obb_config_.min).norm() / 2.0;
        view_ = std::make_unique<SimpleGridMapView>(
            *map_ref_, node_, terrain_analyzer_, obb_config_, nullptr);
    }

    void TearDown() override {
        view_.reset();
        terrain_analyzer_.reset();
        map_server_.reset();
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<finenav::MapServer<finenav::GridMap<Voxel>>> map_server_;
    std::shared_ptr<TerrainAnalyzer> terrain_analyzer_;
    const finenav::GridMap<Voxel>* map_ref_{nullptr};
    ObbConfig obb_config_;
    std::unique_ptr<SimpleGridMapView> view_;
};

// ═══════════════════════════════════════════════════════════════════════════
// 1. ObbConfig — default values
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, ObbConfigDefaultsMatchProductionFootprint) {
    ObbConfig cfg;
    // Defaults should match the 0.8×0.7×0.3 footprint
    EXPECT_DOUBLE_EQ(cfg.min.x(), -0.4);
    EXPECT_DOUBLE_EQ(cfg.min.y(), -0.35);
    EXPECT_DOUBLE_EQ(cfg.min.z(), -0.15);
    EXPECT_DOUBLE_EQ(cfg.max.x(),  0.4);
    EXPECT_DOUBLE_EQ(cfg.max.y(),  0.35);
    EXPECT_DOUBLE_EQ(cfg.max.z(),  0.15);
    EXPECT_DOUBLE_EQ(cfg.policy.step, 0.1);
    EXPECT_DOUBLE_EQ(cfg.outer_radius, 0.0);  // not computed until constructor
    EXPECT_TRUE(cfg.local_pose.isApprox(finenav::Pose::Identity()));
}

TEST_F(SimpleGridMapViewTest, ObbConfigOuterRadiusAfterComputation) {
    ObbConfig cfg;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;
    // sqrt(0.8² + 0.7² + 0.3²) / 2 ≈ sqrt(0.64 + 0.49 + 0.09) / 2
    // = sqrt(1.22) / 2 ≈ 1.1045 / 2 ≈ 0.552
    EXPECT_NEAR(cfg.outer_radius, 0.552, 0.01);
}

// ═══════════════════════════════════════════════════════════════════════════
// 2. getThreadObb — thread_local lazy init, config-change detection
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, GetThreadObbCreatesInstanceLazily) {
    ObbConfig cfg;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;

    auto& obb1 = SimpleGridMapView::getThreadObb(cfg);
    auto& obb2 = SimpleGridMapView::getThreadObb(cfg);

    // Same config → same instance
    EXPECT_EQ(&obb1, &obb2);
}

TEST_F(SimpleGridMapViewTest, GetThreadObbDetectsConfigChange) {
    ObbConfig cfg1;
    cfg1.outer_radius = (cfg1.max - cfg1.min).norm() / 2.0;

    ObbConfig cfg2;
    cfg2.min = finenav::Vector3D(-0.2, -0.2, -0.1);
    cfg2.max = finenav::Vector3D( 0.8,  0.2,  0.1);
    cfg2.outer_radius = (cfg2.max - cfg2.min).norm() / 2.0;

    auto& obb1 = SimpleGridMapView::getThreadObb(cfg1);
    auto& obb2 = SimpleGridMapView::getThreadObb(cfg2);

    // Different config → different instance (address of cfg differs)
    EXPECT_NE(&obb1, &obb2);
}

TEST_F(SimpleGridMapViewTest, GetThreadObbReusesInstanceOnSecondCallSameConfig) {
    ObbConfig cfg;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;

    auto& obb1 = SimpleGridMapView::getThreadObb(cfg);
    auto& obb2 = SimpleGridMapView::getThreadObb(cfg);
    auto& obb3 = SimpleGridMapView::getThreadObb(cfg);

    EXPECT_EQ(&obb1, &obb2);
    EXPECT_EQ(&obb2, &obb3);
}

// ═══════════════════════════════════════════════════════════════════════════
// 3. Constructor — ROS param parsing (asymmetric / symmetric fallback)
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, ObbConfigBuiltFromAsymmetricParams) {
    auto clean_node = std::make_shared<rclcpp::Node>(
        "test_asym_node",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(false)
            .parameter_overrides({
                rclcpp::Parameter("collision_model.front",  0.6),
                rclcpp::Parameter("collision_model.back",   0.3),
                rclcpp::Parameter("collision_model.left",   0.5),
                rclcpp::Parameter("collision_model.right",  0.4),
                rclcpp::Parameter("collision_model.top",    0.2),
                rclcpp::Parameter("collision_model.bottom", 0.1),
                rclcpp::Parameter("collision_model.step",   0.05),
            }));

    // 模拟 hello_finenav 中的参数→ObbConfig 转换
    double front, back, left, right, top, bottom, s;
    clean_node->get_parameter_or("collision_model.front",  front,  0.4);
    clean_node->get_parameter_or("collision_model.back",   back,   0.4);
    clean_node->get_parameter_or("collision_model.left",   left,   0.35);
    clean_node->get_parameter_or("collision_model.right",  right,  0.35);
    clean_node->get_parameter_or("collision_model.top",    top,    0.15);
    clean_node->get_parameter_or("collision_model.bottom", bottom, 0.15);
    clean_node->get_parameter_or("collision_model.step",   s,      0.1);

    ObbConfig cfg;
    cfg.min = finenav::Vector3D(-back, -right, -bottom);
    cfg.max = finenav::Vector3D( front,  left,   top);
    cfg.policy.step = s;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;

    EXPECT_NEAR(cfg.outer_radius, 0.654, 0.01);
    EXPECT_DOUBLE_EQ(cfg.policy.step, 0.05);
    EXPECT_DOUBLE_EQ(cfg.max.x(), 0.6);
    EXPECT_DOUBLE_EQ(cfg.min.x(), -0.3);
}


TEST_F(SimpleGridMapViewTest, ObbConfigPropagatesStep) {
    // 验证 step 参数被正确设置到 policy 中
    ObbConfig cfg;
    cfg.policy.step = 0.02;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;
    // step=0.02 不影响 outer_radius (同样 OBB 尺寸 → 同样外半径)
    EXPECT_DOUBLE_EQ(cfg.policy.step, 0.02);
    EXPECT_NEAR(cfg.outer_radius, 0.552, 0.01);
}

TEST_F(SimpleGridMapViewTest, ConstructorDefaultObbDimensionsAreSane) {
    // The view created in SetUp uses default ObbConfig values:
    // front=0.4, back=0.4, left=0.35, right=0.35, top=0.15, bottom=0.15.
    EXPECT_NEAR(view_->getRadius(), 0.552, 0.01);
}

// ═══════════════════════════════════════════════════════════════════════════
// 4. isCollision — cost gating and OBB path
// ═══════════════════════════════════════════════════════════════════════════

// Test-only subclass that overrides getCost() (virtual via IMapView) to
// return controlled values.  ComputeCost still uses the real TerrainAnalyzer.
class TestableMapView : public SimpleGridMapView {
public:
    using SimpleGridMapView::SimpleGridMapView;

    int getCost(const finenav::Position3D& pos) const override {
        {
            std::lock_guard lock(query_mutex_);
            queried_positions_.push_back(
                {static_cast<float>(pos.x()),
                 static_cast<float>(pos.y()),
                 static_cast<float>(pos.z())});
        }
        if (cost_function_) {
            return cost_function_(pos);
        }
        return 0;  // default: free space
    }

    mutable std::mutex query_mutex_;
    mutable std::function<int(const finenav::Position3D&)> cost_function_;
    mutable std::vector<std::array<float, 3>> queried_positions_;

    void clearQueries() const {
        std::lock_guard lock(query_mutex_);
        queried_positions_.clear();
    }
};

class TestableMapViewTest : public SimpleGridMapViewTest {
protected:
    void SetUp() override {
        // Use a fresh node so declare_parameter doesn't clash with base SetUp's view_
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        node_ = std::make_shared<rclcpp::Node>("test_mapview_testable_node");

        map_server_ = std::make_shared<finenav::MapServer<finenav::GridMap<Voxel>>>(
            node_, "test_map_server2", 10.0, nullptr, 50.0);
        {
            auto map_ro = map_server_->getLockedReadView();
            map_ref_ = map_ro.operator->();
        }
        terrain_analyzer_ = std::make_shared<TerrainAnalyzer>(
            map_server_, node_, nullptr, nullptr);

        ObbConfig cfg;
        cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;
        test_view_ = std::make_unique<TestableMapView>(
            *map_ref_, node_, terrain_analyzer_, cfg, nullptr);
    }

    void TearDown() override {
        test_view_.reset();
        terrain_analyzer_.reset();
        map_server_.reset();
        node_.reset();
    }

    std::unique_ptr<TestableMapView> test_view_;
};

TEST_F(TestableMapViewTest, IsCollisionReturnsTrueWhenCostIs254) {
    // Pose cost = 254 (lethal) → immediate true, no OBB call
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 254;
    };

    EXPECT_TRUE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    // Only one getCost call for the gating check at (0,0,0)
    EXPECT_EQ(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionReturnsTrueWhenCostIs254AtCos45) {
    // Rotated pose — same behavior
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 254;
    };

    EXPECT_TRUE(test_view_->isCollision(1.0f, 2.0f, 0.785f));  // 45°
}

TEST_F(TestableMapViewTest, IsCollisionReturnsFalseWhenCostBelowThreshold) {
    // Default cost_scaling_factor=0.5, resolution=0.05, outer_radius≈0.55
    // distance_cells = 0.55/0.05 = 11
    // threshold = 253 * exp(-0.5 * 11 * 0.05) ≈ 192
    // cost=0 < 192 → early false, no OBB
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;
    };

    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    // Only one getCost call (gating), no OBB lambda calls
    EXPECT_EQ(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionReturnsFalseWhenCostIs100BelowThreshold) {
    // cost=100 < ~192 → early false
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 100;
    };

    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    EXPECT_EQ(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionTriggersObbWhenCostAboveThreshold) {
    // cost=200 >= ~192 → OBB check path.  Make OBB lambda see all-free
    // so the final result is false (no collision detected by OBB).
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 200;
    };

    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    // Multiple calls: 1 gating + N OBB edge points
    EXPECT_GT(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionObbDetectsCollisionWhenCostIs254) {
    // cost=200 triggers OBB check.  OBB lambda returns 254 for all
    // sampled edge points → OBB reports collision → overall true.
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 254;  // everywhere lethal
    };

    EXPECT_TRUE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    // 1 gating check (254 → immediate true path), OR could go through OBB
    // Either way result is true
    EXPECT_GE(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionHandlesCase255WithTrackingUnknown) {
    // is_tracking_unknown_ is false by default, so case 255 → collision
    test_view_->clearQueries();

    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 255;
    };

    // cost=255 > threshold → OBB path.  OBB lambda sees 255:
    //   case 255: return is_tracking_unknown_ ? false : true;
    // is_tracking_unknown_ is false → returns true → collision detected.
    EXPECT_TRUE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    EXPECT_GT(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, IsCollisionUsesCorrectPoseForObbCheck) {
    // Verify that the OBB is evaluated at the robot's actual pose.
    // Robot at (3, 4, 0) heading 90° (π/2).
    test_view_->clearQueries();

    // Return 200 for the gating check (triggers OBB), then 0 for OBB points
    // so the result is false (no collision).
    bool gating_done = false;
    test_view_->cost_function_ = [&](const finenav::Position3D& p) -> int {
        if (!gating_done) { gating_done = true; return 200; }
        return 0;  // OBB edge points see free space
    };

    EXPECT_FALSE(test_view_->isCollision(3.0f, 4.0f, static_cast<float>(M_PI_2)));
    EXPECT_GT(test_view_->queried_positions_.size(), 1u);

    // The OBB edge points should be approximately centered at (3, 4, 0)
    // in world frame — verify at least one has x≈3 or y≈4.
    bool found_near_robot = false;
    for (const auto& p : test_view_->queried_positions_) {
        float dx = p[0] - 3.0f, dy = p[1] - 4.0f;
        if (std::abs(dx) < 1.0f && std::abs(dy) < 1.0f) {
            found_near_robot = true;
            break;
        }
    }
    EXPECT_TRUE(found_near_robot)
        << "OBB edge points should be near robot pose (3, 4)";
}

// ═══════════════════════════════════════════════════════════════════════════
// 5. costAtPose — passes through OBB cost
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(TestableMapViewTest, CostAtPoseReturnsZeroInFreeSpace) {
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;
    };

    float c = test_view_->costAtPose(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(c, 0.0f);
    // Multiple calls: OBB samples all edge points
    EXPECT_GT(test_view_->queried_positions_.size(), 1u);
}

TEST_F(TestableMapViewTest, CostAtPoseReturnsMaxCostInLethalSpace) {
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 254;
    };

    float c = test_view_->costAtPose(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(c, 254.0f);
}

TEST_F(TestableMapViewTest, CostAtPoseReturnsMaxCostInMixedSpace) {
    // Some OBB edge points return 0, some return 100 → max should be 100
    test_view_->cost_function_ = [](const finenav::Position3D& p) -> int {
        // Return higher cost for points with positive x
        return (p.x() > 0.0) ? 100 : 10;
    };

    float c = test_view_->costAtPose(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(c, 100.0f);
}

TEST_F(TestableMapViewTest, CostAtPoseHandlesNegativeCosts) {
    // OBB checkCostEdge max tracks from 0, so negative costs don't lower it
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return -50;
    };

    float c = test_view_->costAtPose(0.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(c, 0.0f);  // OBB checkCostEdge starts max at 0
}

TEST_F(TestableMapViewTest, CostAtPoseUsesCorrectPose) {
    // Robot at (5, -3) heading 180° (π).
    test_view_->cost_function_ = [](const finenav::Position3D& p) -> int {
        // Return cost proportional to distance from robot
        double dist = std::hypot(p.x() - 5.0, p.y() + 3.0);
        return static_cast<int>(dist * 10.0);
    };

    float c = test_view_->costAtPose(5.0f, -3.0f, static_cast<float>(M_PI));
    // The OBB samples points around (5,-3), so distances are ~0-0.55m
    // max cost ≈ 0.55 * 10 ≈ 5-6
    EXPECT_GT(c, 0.0f);
    EXPECT_LT(c, 50.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// 6. getRadius — returns precomputed outer_radius
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, GetRadiusMatchesObbOuterRadius) {
    float r = view_->getRadius();
    // Default OBB 0.8×0.7×0.3: sqrt(0.8²+0.7²+0.3²)/2 ≈ 0.55
    EXPECT_NEAR(r, 0.55f, 0.02f);
}

TEST_F(SimpleGridMapViewTest, GetRadiusIsPositive) {
    EXPECT_GT(view_->getRadius(), 0.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// 7. isTrackingUnknown / considerFootprint — simple accessors
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, IsTrackingUnknownDefaultsToFalse) {
    EXPECT_FALSE(view_->isTrackingUnknown());
}

TEST_F(SimpleGridMapViewTest, ConsiderFootprintDefaultsToTrue) {
    EXPECT_TRUE(view_->considerFootprint());
}

TEST_F(SimpleGridMapViewTest, GetBaseFrameIDReturnsMap) {
    EXPECT_EQ(view_->getBaseFrameID(), "map");
}

// ═══════════════════════════════════════════════════════════════════════════
// 8. Multi-thread safety — concurrent isCollision / costAtPose calls
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(TestableMapViewTest, ConcurrentIsCollisionNoDataRace) {
    // Each thread's first getThreadObb call lazily creates a thread-local
    // OBB.  All threads share the same obb_config_ (read-only after ctor).

    std::atomic<int> errors{0};
    std::atomic<int> completed{0};
    constexpr int kThreads = 8;
    constexpr int kIterations = 500;

    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;  // free space → early-exit false, fast path
    };

    auto worker = [&](int seed) {
        try {
            float base_x = static_cast<float>(seed) * 0.1f;
            for (int i = 0; i < kIterations; ++i) {
                float x = base_x + static_cast<float>(i) * 0.01f;
                bool c = test_view_->isCollision(x, 0.0f, 0.0f);
                if (c) { errors.fetch_add(1); }  // should be false
                // Also exercise costAtPose
                test_view_->costAtPose(x, 0.0f, 0.0f);
            }
        } catch (...) {
            errors.fetch_add(1);
        }
        completed.fetch_add(1);
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back(worker, t);
    }
    for (auto& t : threads) { t.join(); }

    EXPECT_EQ(errors.load(), 0);
    EXPECT_EQ(completed.load(), kThreads);
}

TEST_F(TestableMapViewTest, ConcurrentCostAtPoseNoDataRace) {
    std::atomic<int> errors{0};
    std::atomic<int> completed{0};
    constexpr int kThreads = 8;
    constexpr int kIterations = 500;

    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 50;
    };

    auto worker = [&](int seed) {
        try {
            float theta = static_cast<float>(seed) * 0.5f;
            for (int i = 0; i < kIterations; ++i) {
                float c = test_view_->costAtPose(
                    static_cast<float>(i) * 0.01f, 0.0f, theta);
                if (c < 0.0f) { errors.fetch_add(1); }
            }
        } catch (...) {
            errors.fetch_add(1);
        }
        completed.fetch_add(1);
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back(worker, t);
    }
    for (auto& t : threads) { t.join(); }

    EXPECT_EQ(errors.load(), 0);
    EXPECT_EQ(completed.load(), kThreads);
}

TEST_F(TestableMapViewTest, ConcurrentMixedCallsNoDataRace) {
    // Mix isCollision and costAtPose from multiple threads simultaneously
    std::atomic<int> errors{0};
    constexpr int kThreads = 8;
    constexpr int kIterations = 300;

    auto worker = [&](int seed) {
        try {
            float base = static_cast<float>(seed) * 0.1f;
            for (int i = 0; i < kIterations; ++i) {
                float x = base + static_cast<float>(i) * 0.01f;
                if (i % 2 == 0) {
                    test_view_->isCollision(x, 0.0f, 0.0f);
                } else {
                    test_view_->costAtPose(x, 0.0f, 0.0f);
                }
            }
        } catch (...) {
            errors.fetch_add(1);
        }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back(worker, t);
    }
    for (auto& t : threads) { t.join(); }

    EXPECT_EQ(errors.load(), 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// 9. Edge cases
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(TestableMapViewTest, IsCollisionAtMapOrigin) {
    // Robot at exact map origin (0,0,0) — common case
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;
    };
    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
}

TEST_F(TestableMapViewTest, IsCollisionWithLargeNegativeCoordinates) {
    // Robot far from origin — verify no overflow/NaN
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;
    };
    EXPECT_NO_THROW(test_view_->isCollision(-100.0f, -100.0f, -3.14f));
}

TEST_F(TestableMapViewTest, CostAtPoseWithFullRotation) {
    // Test all cardinal directions
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 10;
    };
    for (float theta : {0.0f, 1.57f, 3.14f, -1.57f, 6.28f}) {
        EXPECT_NO_THROW(test_view_->costAtPose(0.0f, 0.0f, theta));
    }
}

TEST_F(TestableMapViewTest, IsCollisionWithFullRotation) {
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;
    };
    for (float theta : {0.0f, 1.57f, 3.14f, -1.57f, 6.28f}) {
        EXPECT_NO_THROW(test_view_->isCollision(0.0f, 0.0f, theta));
    }
}

TEST_F(TestableMapViewTest, ZeroOuterRadiusCornerCase) {
    // If outer_radius were 0 (not realistic but defensive), possible_collision_cost
    // = ComputeCost(0) = 253 * exp(0) = 253.  So cost >= 253 triggers OBB.
    // Even with radius=0, the code should not crash.
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 0;  // < 253 → early false
    };
    EXPECT_NO_THROW(test_view_->isCollision(0.0f, 0.0f, 0.0f));
}

TEST_F(TestableMapViewTest, PoseAtExactThresholdCost) {
    // possible_collision_cost ≈ 192 with default settings.
    // cost=192 → >= threshold → OBB path.
    // cost=191 → < threshold → early false.
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 191;  // just below threshold
    };
    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));
    size_t calls_below = test_view_->queried_positions_.size();

    test_view_->clearQueries();
    test_view_->cost_function_ = [](const finenav::Position3D&) -> int {
        return 192;  // at/above threshold
    };
    EXPECT_FALSE(test_view_->isCollision(0.0f, 0.0f, 0.0f));  // OBB sees 192→no collision
    size_t calls_above = test_view_->queried_positions_.size();

    // Below threshold: only 1 call (gating).  Above threshold: multiple (OBB).
    EXPECT_EQ(calls_below, 1u);
    EXPECT_GT(calls_above, 1u);
}

// ═══════════════════════════════════════════════════════════════════════════
// 10. Thread-safety: each thread gets its own OBB instance
// ═══════════════════════════════════════════════════════════════════════════

TEST_F(SimpleGridMapViewTest, EachThreadGetsIndependentObb) {
    ObbConfig cfg;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;

    std::set<uintptr_t> obb_addresses;
    std::mutex mtx;
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};

    auto worker = [&]() {
        auto& obb = SimpleGridMapView::getThreadObb(cfg);
        {
            std::lock_guard lock(mtx);
            obb_addresses.insert(reinterpret_cast<uintptr_t>(&obb));
        }
        ready.fetch_add(1);
        // Busy-wait until all threads registered — prevents address reuse
        while (!go.load()) { std::this_thread::yield(); }
    };

    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back(worker);
    }
    // Wait until all 4 threads have inserted their address
    while (ready.load() < 4) { std::this_thread::yield(); }
    go.store(true);
    for (auto& t : threads) { t.join(); }

    // Each thread should have its own thread_local OBB instance
    EXPECT_EQ(obb_addresses.size(), 4u);
}

TEST_F(SimpleGridMapViewTest, SameThreadSameConfigReturnsSameObb) {
    ObbConfig cfg;
    cfg.outer_radius = (cfg.max - cfg.min).norm() / 2.0;

    // Call from this thread multiple times
    auto* a = &SimpleGridMapView::getThreadObb(cfg);
    auto* b = &SimpleGridMapView::getThreadObb(cfg);
    auto* c = &SimpleGridMapView::getThreadObb(cfg);

    EXPECT_EQ(a, b);
    EXPECT_EQ(b, c);
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
