#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <type_traits>
#include <vector>

struct ThreadPool {
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  std::mutex mtx;
  std::condition_variable cv;
  bool stop = false;

  explicit ThreadPool(int n) {
    for (int i = 0; i < n; ++i)
      workers.emplace_back([this] {
        for (;;) {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock(mtx);
            cv.wait(lock, [this] { return stop || !tasks.empty(); });
            if (stop && tasks.empty()) return;
            task = std::move(tasks.front());
            tasks.pop();
          }
          task();
        }
      });
  }

  ~ThreadPool() {
    { std::unique_lock<std::mutex> lock(mtx); stop = true; }
    cv.notify_all();
    for (auto& t : workers) t.join();
  }

  template <typename F>
  auto submit(F fn) -> std::future<std::invoke_result_t<F>> {
    using R = std::invoke_result_t<F>;
    auto task = std::make_shared<std::packaged_task<R()>>(std::move(fn));
    auto fut = task->get_future();
    { std::unique_lock<std::mutex> lock(mtx); tasks.push([task] { (*task)(); }); }
    cv.notify_one();
    return fut;
  }
};
