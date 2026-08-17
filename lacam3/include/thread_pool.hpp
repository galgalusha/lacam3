#pragma once

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

struct PairEntry;

struct ThreadResult { std::vector<PairEntry> entries; long long evaluated; };

struct ThreadPool {
  std::vector<std::thread> workers;
  std::queue<std::packaged_task<ThreadResult()>> tasks;
  std::mutex mtx;
  std::condition_variable cv;
  bool stop = false;

  explicit ThreadPool(int n) {
    for (int i = 0; i < n; ++i)
      workers.emplace_back([this] {
        for (;;) {
          std::packaged_task<ThreadResult()> task;
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

  std::future<ThreadResult> submit(std::function<ThreadResult()> fn) {
    std::packaged_task<ThreadResult()> task(std::move(fn));
    auto fut = task.get_future();
    { std::unique_lock<std::mutex> lock(mtx); tasks.push(std::move(task)); }
    cv.notify_one();
    return fut;
  }
};
