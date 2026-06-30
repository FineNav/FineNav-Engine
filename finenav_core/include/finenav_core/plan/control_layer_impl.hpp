// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of control_layer.hpp.
// It contains the out-of-line template method implementations for ControlLayer.
// Do NOT include this file directly — include control_layer.hpp instead.

#pragma once

namespace finenav {

// ==============================================================================
// Constructor
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
ControlLayer<PlannersT, ExecutionPolicy>::ControlLayer(
    rclcpp::Node::SharedPtr node,
    const std::string& layer_name,
    ExecutionPolicy policy)
    : node_(node), layer_name_(layer_name), policy_(std::move(policy))
{
    rclcpp::NodeOptions pnode_opts;
    pnode_opts.append_parameter_override(
        "use_sim_time",
        node_->get_parameter("use_sim_time").as_bool());
    pnode_ = rclcpp::Node::make_shared(layer_name, node_->get_name(), pnode_opts);
    pnode_thread_ = std::make_unique<NodeThread>(pnode_);

    pnode_->declare_parameter<std::string>("example_param", "hello, world");

    if (!layer_name.empty()) {
        action_server_ = std::make_unique<ActionServer>(
            pnode_,
            layer_name,
            std::bind(&ControlLayer::execute, this));
    }

    std::apply([this](auto&... planners) {
        ([this](auto& p) {
            using T = std::decay_t<decltype(p)>;
            RCLCPP_INFO_STREAM(pnode_->get_logger(), "Loading planner: " << AlgoConfigurator<T>::name);
            keep_alive(AlgoConfigurator<T>::load(pnode_, p));
        }(planners), ...);
    }, planners_instance_);
}

// ==============================================================================
// Topic / Port Setup
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setInputTopic(const std::string& topic) {
    input_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setOutputTopic(const std::string& topic) {
    output_topic_ = topic;
}

template <typename PlannersT, typename ExecutionPolicy>
std::string ControlLayer<PlannersT, ExecutionPolicy>::getName() const {
    return pnode_->get_name();
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::setupDataPorts() {
    if (!input_topic_.empty()) {
        plan_ref_sub_ = pnode_->create_subscription<finenav_msgs::msg::Trajectory>(
            input_topic_, 10,
            std::bind(&ControlLayer::PlanRefCallback, this, std::placeholders::_1));
    }
    if (!output_topic_.empty()) {
        plan_result_pub_ = pnode_->create_publisher<finenav_msgs::msg::Trajectory>(output_topic_, 10);
    }
}

// ==============================================================================
// Utilities
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
template <typename T>
void ControlLayer<PlannersT, ExecutionPolicy>::keep_alive(std::shared_ptr<T> handle) {
    if (handle) {
        param_handles_.push_back(handle);
    }
}

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::PlanRefCallback(
    const finenav_msgs::msg::Trajectory& msg)
{
    std::lock_guard<std::mutex> lock(plan_ref_mutex_);
    plan_ref_msg_ = msg;
}

// ==============================================================================
// Shared Helpers
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::resetActivePlanner() {
    std::apply([&](auto&... planners) {
        ([&](auto& p) {
            using PlannerT = std::decay_t<decltype(p)>;
            if (AlgoConfigurator<PlannerT>::name == active_planner_id_) {
                if constexpr (requires { p.reset(); }) {
                    p.reset();
                    RCLCPP_INFO(pnode_->get_logger(),
                        "Planner '%s' reset at episode boundary.", active_planner_id_.c_str());
                }
            }
        }(planners), ...);
    }, planners_instance_);
}

template <typename PlannersT, typename ExecutionPolicy>
bool ControlLayer<PlannersT, ExecutionPolicy>::planOnce(geometry_msgs::msg::Pose goal_pose) {
    if (!map_view_factory_) {
        RCLCPP_ERROR(pnode_->get_logger(), "MapViewFactory not set!");
        return false;
    }
    if (!robot_state_provider_) {
        RCLCPP_ERROR(pnode_->get_logger(),
            "RobotStateProvider not set! Call set_robot_state_provider() first.");
        return false;
    }

    finenav_msgs::msg::RobotState current_robot_state = robot_state_provider_();

    finenav_msgs::msg::Trajectory current_ref_msg;
    {
        std::lock_guard<std::mutex> lock(plan_ref_mutex_);
        current_ref_msg = plan_ref_msg_;
    }

    bool plan_success  = false;
    bool planner_found = false;
    finenav_msgs::msg::Trajectory output_traj_msg;

    auto try_run_planner = [&](auto& planner) -> bool {
        using PlannerT = std::decay_t<decltype(planner)>;
        if (AlgoConfigurator<PlannerT>::name != active_planner_id_) {
            return false;
        }

        auto opt_ctx = [&]() -> std::optional<PlanningContext<PlannerT>> {
            PrePlanContext pre_ctx{
                active_planner_id_, current_robot_state, goal_pose, current_ref_msg};

            if (on_pre_plan_ && !on_pre_plan_(pre_ctx)) {
                return std::nullopt; // 用户拦截，中止此次规划
            }

            PlanningContext<PlannerT> ctx;
            ctx.robot_state = pre_ctx.robot_state;
            ctx.goal_pose   = pre_ctx.goal_pose;
            fromROSMsg(std::move(pre_ctx.ref_traj), ctx.ref_traj);
            return ctx;
        }();

        if (!opt_ctx.has_value()) {
            return true; // 短路遍历
        }

        auto& ctx   = opt_ctx.value();
        auto  bound = map_view_factory_();

        using ExpectedPtrType = std::shared_ptr<typename PlannerT::IMapView>;
        if (auto* ptr = std::get_if<ExpectedPtrType>(&bound.view_variant)) {
            if (*ptr) {
                // bound.lock_guard 持有读锁，plan() 调用期间地图不被写入
                auto result = planner.plan(ctx, **ptr);
                if (result.has_value()) {
                    plan_success = true;
                    toROSMsg(std::move(result.value()), output_traj_msg);
                } else {
                    RCLCPP_WARN(pnode_->get_logger(), "plan() returned nullopt.");
                }
            } else {
                RCLCPP_ERROR(pnode_->get_logger(),
                    "MapView ptr is null for planner '%s'.", active_planner_id_.c_str());
            }
        } else {
            RCLCPP_ERROR(pnode_->get_logger(),
                "MapView type mismatch for planner '%s'.", active_planner_id_.c_str());
        }

        if (on_post_plan_) {
            PostPlanContext post_ctx{active_planner_id_, plan_success, output_traj_msg};
            plan_success = on_post_plan_(post_ctx);
        }

        return true; // 短路遍历
    };

    std::apply([&](auto&... planners) {
        planner_found = (try_run_planner(planners) || ...);
    }, planners_instance_);

    if (!planner_found) {
        RCLCPP_ERROR_STREAM(pnode_->get_logger(),
            "Planner '" << active_planner_id_ << "' not found.");
        return false;
    }

    if (plan_success && plan_result_pub_) {
        plan_result_pub_->publish(output_traj_msg);
    }

    return plan_success;
}

// ==============================================================================
// execute() — Episode Entry-Point
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::execute() {
    RCLCPP_INFO(pnode_->get_logger(), "New goal received — starting episode.");

    auto goal          = action_server_->get_current_goal();
    active_planner_id_ = goal->planner_id;

    // Episode 边界：在任何 plan() 调用前重置规划器内部状态
    resetActivePlanner();

    // 按策略执行
    executeImpl(policy_);
}

// ==============================================================================
// executeImpl — SingleShotPolicy
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(SingleShotPolicy&) {
    auto goal_pose = action_server_->get_current_goal()->goal;

    if (action_server_->is_cancel_requested()) {
        RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal cancelled before plan.");
        action_server_->terminate_all();
        return;
    }

    if (!planOnce(goal_pose)) {
        RCLCPP_ERROR(pnode_->get_logger(), "SingleShot: plan() failed.");
        action_server_->terminate_all();
        return;
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "SingleShot: goal succeeded.");
}

// ==============================================================================
// executeImpl — TrackingPolicy
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
void ControlLayer<PlannersT, ExecutionPolicy>::executeImpl(TrackingPolicy& policy) {
    auto goal_pose = action_server_->get_current_goal()->goal;
    rclcpp::Rate rate(policy.frequency_hz);

    RCLCPP_INFO(pnode_->get_logger(),
        "Tracking: control loop started at %.1f Hz.", policy.frequency_hz);

    while (rclcpp::ok()) {
        if (action_server_->is_cancel_requested()) {
            // BT 驱动的正常取消（如 IsGoalReached/IsStuck 触发），不代表错误。
            RCLCPP_INFO(pnode_->get_logger(),
                "Tracking: goal cancelled by BT (normal termination).");
            action_server_->terminate_all();
            return;
        }

        // 检测 BT 侧通过 shouldResendGoal() 发来的新 goal（进入 pending slot）。
        // 去重：若 pending goal 与当前 goal 位姿相同，丢弃而非抢占，避免
        //   abort → BT FAILURE → RetryUntilSuccessful 重发 → 再次 pending 的乒乓死循环。
        if (action_server_->is_preempt_requested()) {
            auto pending_goal = action_server_->get_pending_goal();
            if (pending_goal) {
                constexpr double kPosEps = 0.01;   // 1 cm
                constexpr double kYawEps = 0.01;   // ~0.6°
                const auto& pg = pending_goal->goal;
                const auto& cg = goal_pose;
                double dx = pg.position.x - cg.position.x;
                double dy = pg.position.y - cg.position.y;
                double dyaw = std::abs(tf2::getYaw(pg.orientation) - tf2::getYaw(cg.orientation));
                // wrap dyaw to [-π, π] for comparison
                if (dyaw > M_PI) dyaw = 2.0 * M_PI - dyaw;

                if (dx * dx + dy * dy < kPosEps * kPosEps && dyaw < kYawEps) {
                    // 重复 goal，直接丢弃，保持当前追踪状态不变
                    RCLCPP_DEBUG(pnode_->get_logger(),
                        "Tracking: duplicate pending goal discarded (%.2f, %.2f).",
                        pg.position.x, pg.position.y);
                    action_server_->discard_pending_goal();
                } else {
                    // 真正的新 goal，执行抢占
                    auto new_goal = action_server_->accept_pending_goal();
                    if (new_goal) {
                        goal_pose = new_goal->goal;
                        RCLCPP_INFO(pnode_->get_logger(),
                            "Tracking: goal preempted in-loop (%.2f, %.2f) — MPPI warm-start preserved.",
                            goal_pose.position.x, goal_pose.position.y);
                    }
                }
            }
        }

        if (!planOnce(goal_pose)) {
            RCLCPP_ERROR(pnode_->get_logger(), "Tracking: plan() failed — aborting.");
            action_server_->terminate_all();
            return;
        }


        rate.sleep();
    }

    action_server_->succeeded_current();
    RCLCPP_INFO(pnode_->get_logger(), "Tracking: goal succeeded.");
}

// ==============================================================================
// bind_map_view
// ==============================================================================

template <typename PlannersT, typename ExecutionPolicy>
template <MapConcept MapT, typename BuilderFn>
requires std::is_invocable_v<BuilderFn, const MapT&> &&
         std::convertible_to<std::invoke_result_t<BuilderFn, const MapT&>,
                             typename ControlLayer<PlannersT, ExecutionPolicy>::MapViewsVariant>
void ControlLayer<PlannersT, ExecutionPolicy>::bind_map_view(
    std::shared_ptr<MapServer<MapT>> server,
    BuilderFn&& builder)
{
    std::function<MapViewsVariant(const MapT&)> erased_builder =
        std::forward<BuilderFn>(builder);
    std::weak_ptr<MapServer<MapT>> weak_server = server;

    map_view_factory_ = [weak_server, erased_builder = std::move(erased_builder)]() -> BoundMapView
    {
        auto srv = weak_server.lock();
        if (!srv) {
            return BoundMapView{};
        }

        auto locked = std::make_shared<LockedMapRO<MapT>>(srv->getLockedReadView());
        MapViewsVariant view_variant = erased_builder(locked->get());
        return BoundMapView{std::move(view_variant), std::move(locked)};
    };
}

} // namespace finenav

