// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// Lightweight static thread pool for MPPI batch-level parallelism.
// Zero external dependencies — uses only C++20 std::thread / std::jthread.

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace mppi {

class ThreadPool {
public:
    /// @param num_threads  工作线程数。0 = 自动检测 hardware_concurrency()。
    explicit ThreadPool(size_t num_threads = 0)
        : num_threads_(num_threads > 0 ? num_threads
                                       : std::max(1u, std::thread::hardware_concurrency()))
    {
        for (size_t t = 0; t < num_threads_; ++t) {
            workers_.emplace_back([this, t] { worker_loop(t); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard lock(mutex_);
            stop_ = true;
            ++generation_;  // wake all
        }
        cv_.notify_all();
        // jthread destructor auto-joins
    }

    size_t num_threads() const { return num_threads_; }

    /// @brief 并行 for 循环。主线程参与计算，调用线程阻塞直到所有任务完成。
    /// @param func  void(size_t index, size_t thread_id) — index 在 [begin,end)，
    ///              thread_id 在 [0, num_threads]（num_threads 为主线程）。
    template <typename Func>
    void parallel_for(size_t begin, size_t end, Func&& func) {
        const size_t count = end - begin;
        if (count == 0) return;

        task_func_ = [&func](size_t i, size_t tid) { func(i, tid); };
        task_begin_ = begin;
        task_end_   = end;
        workers_done_ = 0;

        {
            std::lock_guard lock(mutex_);
            ++generation_;   // 新的一轮，唤醒所有 worker
        }
        cv_.notify_all();

        // 主线程参与计算
        execute_range(/*thread_id=*/num_threads_);

        // 等待所有 worker 完成
        {
            std::unique_lock lock(mutex_);
            cv_done_.wait(lock, [this] { return workers_done_ == num_threads_; });
        }
    }

private:
    void worker_loop(size_t thread_id) {
        size_t my_generation = 0;
        while (true) {
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this, &my_generation] {
                    return generation_ > my_generation || stop_;
                });
                if (stop_) return;
                my_generation = generation_;
            }

            execute_range(thread_id);

            {
                std::lock_guard lock(mutex_);
                ++workers_done_;
            }
            cv_done_.notify_one();
        }
    }

    void execute_range(size_t thread_id) {
        if (!task_func_) return;
        const size_t begin      = task_begin_;
        const size_t end        = task_end_;
        const size_t count      = end - begin;
        const size_t total_thr  = num_threads_ + 1;           // workers + main
        const size_t chunk      = (count + total_thr - 1) / total_thr;
        const size_t my_begin   = begin + thread_id * chunk;
        const size_t my_end     = std::min(my_begin + chunk, end);

        for (size_t i = my_begin; i < my_end; ++i) {
            task_func_(i, thread_id);
        }
    }

    size_t                          num_threads_;
    std::vector<std::jthread>       workers_;

    std::mutex                      mutex_;
    std::condition_variable         cv_;        // 唤醒 worker
    std::condition_variable         cv_done_;   // 通知主线程全部完成
    size_t                          generation_{0};
    size_t                          workers_done_{0};
    bool                            stop_{false};

    std::function<void(size_t, size_t)> task_func_;
    size_t                          task_begin_{0};
    size_t                          task_end_{0};
};

}  // namespace mppi
