// Dummy, intentionally naive pair-penalty DB generator.
//
// Unlike HorizonPairDBGenerator, which generates penalties for pairs of MDDs
// (an abstraction that groups together every agent sharing the same MDD),
// this "hard way" generator iterates over every possible pair of agents --
// i.e. every (start1, goal1) x (start2, goal2) combination, exactly like the
// nested loops in cmp_db.cpp -- to demonstrate how much more expensive that
// approach is, even on tiny maps.
#include <lacam.hpp>
#include <thread_pool.hpp>
#include <horizon_pair_db_file.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

static const int NUM_OF_THREADS = 7;

struct Agent {
  Vertex *start;
  Vertex *goal;
};

#pragma pack(push, 1)
struct PairRecord {
  uint16_t s1, g1, s2, g2;
  uint8_t penalty;
};
#pragma pack(pop)

// Returns 1 once every 1000 invocations and 0 otherwise, regardless of the
// agents passed in. Only the invocation count matters -- this is a dummy
// stand-in for a real conflict-penalty computation. The counter is owned by
// the caller (one per task/thread) so this hot path never touches shared or
// atomic state.
static uint8_t calc_pair_penalty(const Agent &a1, const Agent &a2, uint64_t &invocation_count)
{
  (void)a1;
  (void)a2;
  return (++invocation_count % 1000 == 0) ? 1 : 0;
}

// Runs a dedicated thread that redraws the progress bar at a fixed rate while
// workers only bump the atomic counter. Mirrors the strategy used by
// HorizonPairDBGenerator.
template <typename T>
static void wait_with_progress(std::vector<std::future<void>> &futures, const std::atomic<T> &done, size_t total)
{
  std::mutex mtx;
  std::condition_variable cv;
  bool stop = false;

  std::thread printer([&]() {
    std::unique_lock lock(mtx);
    while (!stop) {
      print_horizon_pair_db_progress_bar(done.load(), total);
      cv.wait_for(lock, std::chrono::milliseconds(100), [&]() { return stop; });
    }
  });

  auto stop_printer = [&]() {
    {
      std::lock_guard lock(mtx);
      stop = true;
    }
    cv.notify_one();
    printer.join();
  };

  try {
    for (auto &fut : futures) fut.get();
  } catch (...) {
    stop_printer();
    throw;
  }
  stop_printer();
  print_horizon_pair_db_progress_bar(total, total);
}

int main(int argc, char *argv[])
{
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <width> <height>" << std::endl;
    return 1;
  }
  const int W = std::stoi(argv[1]);
  const int H = std::stoi(argv[2]);

  std::vector<std::string> grid(H, std::string(W, '.'));
  Graph *G = new Graph(grid);
  const size_t V_SIZE = G->V.size();

  const std::string out_path = "dummy_db_the_hard_way_" + std::to_string(W) + "x" +
                                std::to_string(H) + ".bin";
  std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
  if (!out) {
    std::cerr << "failed to open output file " << out_path << std::endl;
    delete G;
    return 1;
  }

  const auto start_time = std::chrono::steady_clock::now();

  std::mutex out_mtx;
  std::atomic<size_t> done = 0;
  const size_t total = V_SIZE * V_SIZE;  // number of (start1, goal1) agents

  // Task processes an ENTIRE start1 vertex, exactly like the per-start-vertex
  // tasks in HorizonPairDBGenerator::generate_mdds.
  auto task = [&](int s1_i) {
    Vertex *start1 = G->V[s1_i];
    std::vector<PairRecord> batch;
    batch.reserve(V_SIZE * V_SIZE);
    // Owned solely by this task (and hence by whichever thread runs it), so
    // calc_pair_penalty never needs atomics or locking.
    uint64_t invocation_count = 0;

    for (auto goal1 : G->V) {
      const Agent a1{start1, goal1};
      batch.clear();

      for (auto start2 : G->V) {
        for (auto goal2 : G->V) {
          const Agent a2{start2, goal2};
          const uint8_t penalty = calc_pair_penalty(a1, a2, invocation_count);
          batch.push_back({static_cast<uint16_t>(start1->id), static_cast<uint16_t>(goal1->id),
                            static_cast<uint16_t>(start2->id), static_cast<uint16_t>(goal2->id),
                            penalty});
        }
      }

      // {
      //   std::lock_guard lock(out_mtx);
      //   out.write(reinterpret_cast<const char *>(batch.data()),
      //              static_cast<std::streamsize>(batch.size() * sizeof(PairRecord)));
      // }
      ++done;
    }
  };

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<void>> futures;
  futures.reserve(V_SIZE);
  for (int s1_i = 0; s1_i < (int)V_SIZE; s1_i++) {
    futures.push_back(pool.submit([&task, s1_i]() { task(s1_i); }));
  }

  wait_with_progress(futures, done, total);
  std::cout << std::endl;

  out.close();
  delete G;

  const auto end_time = std::chrono::steady_clock::now();
  const double secs = std::chrono::duration<double>(end_time - start_time).count();
  std::cout << "Generated dummy DB the hard way for " << W << "x" << H << " map in " << secs
            << " seconds." << std::endl;

  return 0;
}
