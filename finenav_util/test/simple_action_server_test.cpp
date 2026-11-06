#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <finenav_msgs/action/compute_plan.hpp>
#include "finenav_util/simple_action_server.hpp"

using namespace std::chrono_literals;

namespace {

using Action = finenav_msgs::action::ComputePlan;
using Server = finenav::SimpleActionServer<Action>;
using Client = rclcpp_action::Client<Action>;
using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

class ActionHarness {
public:
    explicit ActionHarness(std::function<void()> execute)
    {
        const auto suffix = std::to_string(next_id_++);
        const auto action_name = "test_action_" + suffix;
        server_node_ = std::make_shared<rclcpp::Node>("test_server_" + suffix);
        client_node_ = std::make_shared<rclcpp::Node>("test_client_" + suffix);
        server_ = std::make_unique<Server>(server_node_, action_name, std::move(execute));
        client_ = rclcpp_action::create_client<Action>(client_node_, action_name);
        server_executor_.add_node(server_node_);
        client_executor_.add_node(client_node_);
        server_thread_ = std::thread([this] { server_executor_.spin(); });
    }

    ~ActionHarness()
    {
        server_executor_.cancel();
        server_thread_.join();
        server_.reset();
        client_executor_.remove_node(client_node_);
        server_executor_.remove_node(server_node_);
    }

    Server& server() { return *server_; }

    auto send(int id)
    {
        Action::Goal goal;
        goal.planner_id = std::to_string(id);
        return client_->async_send_goal(goal);
    }

    GoalHandle::SharedPtr accepted(std::shared_future<GoalHandle::SharedPtr>& future)
    {
        if (client_executor_.spin_until_future_complete(future, 3s) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return nullptr;
        }
        return future.get();
    }

    rclcpp_action::ResultCode result(const GoalHandle::SharedPtr& goal)
    {
        auto future = client_->async_get_result(goal);
        if (client_executor_.spin_until_future_complete(future, 3s) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            return rclcpp_action::ResultCode::UNKNOWN;
        }
        return future.get().code;
    }

    bool cancel(const GoalHandle::SharedPtr& goal)
    {
        auto future = client_->async_cancel_goal(goal);
        return client_executor_.spin_until_future_complete(future, 3s) ==
               rclcpp::FutureReturnCode::SUCCESS;
    }

    bool waitForPending(const std::string& id)
    {
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline) {
            const auto goal = server_->get_pending_goal();
            if (goal && goal->planner_id == id) return true;
            std::this_thread::sleep_for(1ms);
        }
        return false;
    }

private:
    static inline std::atomic<int> next_id_{0};
    rclcpp::Node::SharedPtr server_node_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<Server> server_;
    Client::SharedPtr client_;
    rclcpp::executors::SingleThreadedExecutor server_executor_;
    rclcpp::executors::SingleThreadedExecutor client_executor_;
    std::thread server_thread_;
};

class SimpleActionServerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(SimpleActionServerTest, SequentialGoalsEachExecuteAndSucceed)
{
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        executed.push_back(goal->planner_id);
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        harness_ptr->server().succeeded_current();
    });
    harness_ptr = &harness;

    for (int id = 0; id < 20; ++id) {
        auto future = harness.send(id);
        auto goal = harness.accepted(future);
        ASSERT_NE(goal, nullptr) << "goal " << id << " was not accepted";
        ASSERT_EQ(harness.result(goal), rclcpp_action::ResultCode::SUCCEEDED)
            << "goal " << id << " did not finish";
    }
    ASSERT_EQ(executed.size(), 20u);
    for (int id = 0; id < 20; ++id) {
        EXPECT_EQ(executed[id], std::to_string(id));
    }
    for (int i = 0; i < 10; ++i) {
        auto future = harness.send(7);
        auto goal = harness.accepted(future);
        ASSERT_NE(goal, nullptr);
        ASSERT_EQ(harness.result(goal), rclcpp_action::ResultCode::SUCCEEDED);
    }
    ASSERT_EQ(executed.size(), 30u);
    for (int i = 20; i < 30; ++i) {
        EXPECT_EQ(executed[i], "7");
    }
}

TEST_F(SimpleActionServerTest, NewGoalWhileCompletedWorkerIsFinishing)
{
    std::mutex mutex;
    std::condition_variable cv;
    bool first_completed = false;
    bool release_first = false;
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        {
            std::lock_guard<std::mutex> lock(mutex);
            executed.push_back(goal->planner_id);
        }
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        harness_ptr->server().succeeded_current();
        if (goal->planner_id == "1") {
            std::unique_lock<std::mutex> lock(mutex);
            first_completed = true;
            cv.notify_all();
            cv.wait_for(lock, 3s, [&] { return release_first; });
        }
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_completed; }));
    }
    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::SUCCEEDED);

    auto second_future = harness.send(2);
    auto second = harness.accepted(second_future);
    ASSERT_NE(second, nullptr);
    std::this_thread::sleep_for(10ms);
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_first = true;
    }
    cv.notify_all();
    EXPECT_EQ(harness.result(second), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "2"}));
}

TEST_F(SimpleActionServerTest, NewestPendingGoalRunsAfterCurrentCompletes)
{
    std::mutex mutex;
    std::condition_variable cv;
    bool first_started = false;
    bool first_completed = false;
    bool release_first = false;
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        {
            std::lock_guard<std::mutex> lock(mutex);
            executed.push_back(goal->planner_id);
            if (goal->planner_id == "1") first_started = true;
        }
        cv.notify_all();
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        if (goal->planner_id == "1") {
            const auto deadline = std::chrono::steady_clock::now() + 3s;
            while (!harness_ptr->server().is_preempt_requested() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            harness_ptr->server().succeeded_current();
            std::unique_lock<std::mutex> lock(mutex);
            first_completed = true;
            cv.notify_all();
            cv.wait_for(lock, 3s, [&] { return release_first; });
        } else {
            harness_ptr->server().succeeded_current();
        }
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_started; }));
    }
    auto pending_future = harness.send(2);
    auto pending = harness.accepted(pending_future);
    ASSERT_NE(pending, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_completed; }));
    }
    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::SUCCEEDED);

    auto latest_future = harness.send(3);
    auto latest = harness.accepted(latest_future);
    ASSERT_NE(latest, nullptr);
    const bool latest_is_pending = harness.waitForPending("3");
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_first = true;
    }
    cv.notify_all();

    ASSERT_TRUE(latest_is_pending);
    EXPECT_EQ(harness.result(pending), rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(harness.result(latest), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "3"}));
}

TEST_F(SimpleActionServerTest, NewGoalRunsAfterCancellation)
{
    std::mutex mutex;
    std::condition_variable cv;
    bool first_started = false;
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        {
            std::lock_guard<std::mutex> lock(mutex);
            executed.push_back(goal->planner_id);
            if (goal->planner_id == "1") first_started = true;
        }
        cv.notify_all();
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        if (goal->planner_id == "1") {
            const auto deadline = std::chrono::steady_clock::now() + 3s;
            while (!harness_ptr->server().is_cancel_requested() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            harness_ptr->server().terminate_all();
        } else {
            harness_ptr->server().succeeded_current();
        }
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_started; }));
    }
    ASSERT_TRUE(harness.cancel(first));
    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::ABORTED);

    auto second_future = harness.send(2);
    auto second = harness.accepted(second_future);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(harness.result(second), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "2"}));
}

TEST_F(SimpleActionServerTest, ActiveGoalIsAbortedBeforePendingGoalRuns)
{
    std::mutex mutex;
    std::condition_variable cv;
    bool first_started = false;
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        {
            std::lock_guard<std::mutex> lock(mutex);
            executed.push_back(goal->planner_id);
            if (goal->planner_id == "1") first_started = true;
        }
        cv.notify_all();
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        if (goal->planner_id == "1") {
            const auto deadline = std::chrono::steady_clock::now() + 3s;
            while (!harness_ptr->server().is_preempt_requested() &&
                   std::chrono::steady_clock::now() < deadline) {
                std::this_thread::sleep_for(1ms);
            }
            return;
        }
        harness_ptr->server().succeeded_current();
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_started; }));
    }
    auto second_future = harness.send(2);
    auto second = harness.accepted(second_future);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(harness.result(second), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "2"}));
}

TEST_F(SimpleActionServerTest, RepeatedPendingGoalsOnlyExecuteLatest)
{
    std::mutex mutex;
    std::condition_variable cv;
    bool first_started = false;
    bool release_first = false;
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        {
            std::lock_guard<std::mutex> lock(mutex);
            executed.push_back(goal->planner_id);
            if (goal->planner_id == "1") first_started = true;
        }
        cv.notify_all();
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        if (goal->planner_id == "1") {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait_for(lock, 3s, [&] { return release_first; });
            return;
        }
        harness_ptr->server().succeeded_current();
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(cv.wait_for(lock, 3s, [&] { return first_started; }));
    }

    std::vector<GoalHandle::SharedPtr> pending;
    for (int id = 2; id <= 5; ++id) {
        auto future = harness.send(id);
        auto goal = harness.accepted(future);
        ASSERT_NE(goal, nullptr);
        pending.push_back(goal);
        ASSERT_TRUE(harness.waitForPending(std::to_string(id)));
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_first = true;
    }
    cv.notify_all();

    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::ABORTED);
    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(harness.result(pending[i]), rclcpp_action::ResultCode::ABORTED);
    }
    EXPECT_EQ(harness.result(pending.back()), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "5"}));
}

TEST_F(SimpleActionServerTest, NewGoalRunsAfterExecutionException)
{
    std::vector<std::string> executed;
    ActionHarness* harness_ptr = nullptr;
    ActionHarness harness([&] {
        const auto goal = harness_ptr->server().get_current_goal();
        executed.push_back(goal->planner_id);
        RCLCPP_INFO(rclcpp::get_logger("simple_action_server_test"),
            "Executing goal %s", goal->planner_id.c_str());
        if (goal->planner_id == "1") throw std::runtime_error("test failure");
        harness_ptr->server().succeeded_current();
    });
    harness_ptr = &harness;

    auto first_future = harness.send(1);
    auto first = harness.accepted(first_future);
    ASSERT_NE(first, nullptr);
    EXPECT_EQ(harness.result(first), rclcpp_action::ResultCode::ABORTED);

    auto second_future = harness.send(2);
    auto second = harness.accepted(second_future);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(harness.result(second), rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_EQ(executed, (std::vector<std::string>{"1", "2"}));
}

}  // namespace
