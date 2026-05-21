// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "finenav_msgs/action/compute_path.hpp"
#include "finenav_msgs/action/compute_plan.hpp"
#include "finenav_msgs/msg/trajectory.hpp"
#include "finenav_route/route_path_planner.hpp"

namespace finenav
{

class ComputePathBridge : public rclcpp::Node
{
public:
    using ComputePath = finenav_msgs::action::ComputePath;
    using ComputePlan = finenav_msgs::action::ComputePlan;
    using ComputePathGoalHandle = rclcpp_action::ServerGoalHandle<ComputePath>;

    ComputePathBridge()
    : rclcpp::Node("compute_path_bridge")
    {
        action_name_ = declare_parameter<std::string>("action_name", "compute_path");
        controller_action_name_ =
            declare_parameter<std::string>("controller_action_name", "mppi_layer");
        path_topic_ = declare_parameter<std::string>("path_topic", "line_path");
        server_timeout_s_ = declare_parameter<double>("server_timeout_s", 2.0);

        path_pub_ = create_publisher<finenav_msgs::msg::Trajectory>(path_topic_, 10);
        controller_client_ =
            rclcpp_action::create_client<ComputePlan>(this, controller_action_name_);

        action_server_ = rclcpp_action::create_server<ComputePath>(
            this,
            action_name_,
            std::bind(&ComputePathBridge::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ComputePathBridge::handleCancel, this, std::placeholders::_1),
            std::bind(&ComputePathBridge::handleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "ComputePath bridge ready: action='%s', path_topic='%s', controller_action='%s'",
            action_name_.c_str(), path_topic_.c_str(), controller_action_name_.c_str());
    }

private:
    template <typename FutureT>
    bool waitForFuture(FutureT & future, const std::shared_ptr<ComputePathGoalHandle> & goal_handle)
    {
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                return false;
            }
            if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                return true;
            }
        }
        return false;
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ComputePath::Goal> goal)
    {
        if (goal->graph_json.empty())
        {
            RCLCPP_WARN(get_logger(), "Rejecting ComputePath goal without route graph json.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<ComputePathGoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<ComputePathGoalHandle> goal_handle)
    {
        std::thread{std::bind(&ComputePathBridge::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<ComputePathGoalHandle> goal_handle)
    {
        const auto start_time = now();
        const auto goal = goal_handle->get_goal();

        finenav_msgs::msg::Trajectory path;
        try {
            path = route_path_planner_.plan(
                goal->graph_json, goal->start_node_id, goal->goal_node_id);
        } catch (const std::exception & ex) {
            RCLCPP_ERROR(get_logger(), "Route path planning failed: %s", ex.what());
            goal_handle->abort(makeResult(start_time));
            return;
        }

        if ((path.valid_fields & finenav_msgs::msg::Trajectory::TRAJ_POSE) == 0 ||
            path.poses.empty())
        {
            RCLCPP_ERROR(get_logger(), "Route path planning produced an empty pose trajectory.");
            goal_handle->abort(makeResult(start_time));
            return;
        }

        path_pub_->publish(path);
        RCLCPP_INFO(
            get_logger(),
            "Published route reference trajectory with %zu poses from node %u to node %u.",
            path.poses.size(), goal->start_node_id, goal->goal_node_id);

        const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(server_timeout_s_));
        if (!controller_client_->wait_for_action_server(timeout)) {
            RCLCPP_ERROR(
                get_logger(),
                "Controller action server '%s' is not available.",
                controller_action_name_.c_str());
            goal_handle->abort(makeResult(start_time));
            return;
        }

        ComputePlan::Goal controller_goal;
        controller_goal.goal = path.poses.back();
        controller_goal.planner_id = goal->planner_id;

        auto controller_goal_future = controller_client_->async_send_goal(controller_goal);
        if (!waitForFuture(controller_goal_future, goal_handle)) {
            goal_handle->canceled(makeResult(start_time));
            return;
        }

        auto controller_goal_handle = controller_goal_future.get();
        if (!controller_goal_handle) {
            RCLCPP_ERROR(get_logger(), "Controller rejected forwarded ComputePlan goal.");
            goal_handle->abort(makeResult(start_time));
            return;
        }

        auto result_future = controller_client_->async_get_result(controller_goal_handle);
        if (!waitForFuture(result_future, goal_handle)) {
            controller_client_->async_cancel_goal(controller_goal_handle);
            goal_handle->canceled(makeResult(start_time));
            return;
        }

        const auto wrapped_result = result_future.get();
        auto bridge_result = makeResult(start_time);
        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                bridge_result->path = path;
                goal_handle->succeed(bridge_result);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                goal_handle->canceled(bridge_result);
                break;
            default:
                goal_handle->abort(bridge_result);
                break;
        }
    }

    std::shared_ptr<ComputePath::Result> makeResult(const rclcpp::Time & start_time)
    {
        auto result = std::make_shared<ComputePath::Result>();
        const auto duration = now() - start_time;
        const auto total_ns = duration.nanoseconds();
        result->planning_time.sec = static_cast<int32_t>(total_ns / 1000000000);
        result->planning_time.nanosec = static_cast<uint32_t>(total_ns % 1000000000);
        return result;
    }

    std::string action_name_;
    std::string controller_action_name_;
    std::string path_topic_;
    double server_timeout_s_{2.0};

    rclcpp::Publisher<finenav_msgs::msg::Trajectory>::SharedPtr path_pub_;
    rclcpp_action::Client<ComputePlan>::SharedPtr controller_client_;
    rclcpp_action::Server<ComputePath>::SharedPtr action_server_;
    finenav_route::RoutePathPlanner route_path_planner_;
};

}  // namespace finenav

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<finenav::ComputePathBridge>());
    rclcpp::shutdown();
    return 0;
}
