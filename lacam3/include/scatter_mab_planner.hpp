#pragma once

#include "dist_table.hpp"
#include "graph.hpp"
#include "heuristic.hpp"
#include "hnode.hpp"
#include "instance.hpp"
#include "metrics.hpp"
#include "params.hpp"
#include "pibt.hpp"
#include "refiner.hpp"
#include "scatter.hpp"
#include "translator.hpp"
#include "utils.hpp"

#include <condition_variable>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <thread>

struct ScatterConfig {
  int margin;
};

struct EpochResult {
  bool success;
  int cost;
};

// Fixed-size thread pool: threads are created once and reused.
struct ThreadPool {
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> tasks;
  std::mutex mtx;
  std::condition_variable cv;
  bool stop = false;

  explicit ThreadPool(int n)
  {
    for (int i = 0; i < n; ++i) {
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
  }

  ~ThreadPool()
  {
    { std::unique_lock<std::mutex> lock(mtx); stop = true; }
    cv.notify_all();
    for (auto &w : workers) w.join();
  }

  template <typename F>
  std::future<std::invoke_result_t<F>> submit(F &&f)
  {
    using R = std::invoke_result_t<F>;
    auto task = std::make_shared<std::packaged_task<R()>>(std::forward<F>(f));
    std::future<R> fut = task->get_future();
    { std::unique_lock<std::mutex> lock(mtx); tasks.emplace([task] { (*task)(); }); }
    cv.notify_one();
    return fut;
  }
};

struct Arm {
  HNode* H_init = nullptr;
  std::unique_ptr<std::mutex> h_init_mutex = std::make_unique<std::mutex>();
  int id;
  IScatter *scatter;
  std::vector<int> costs;  // protected by result_mutex in explore_scatters
  int num_of_runs = 0;     // protected by result_mutex in explore_scatters
  bool active = true;      // set to false when eliminated
};

struct ScatterMABPlanner {
  static constexpr int EXPLORATION_THREADS = 8;
  const Instance *ins;
  const Deadline *deadline;
  const int seed;
  std::mt19937 MT;
  const int verbose;
  const int depth;

  // solver utils
  const int N;  // number of agents
  const int V_size;
  int LB;
  DistTable *D;
  bool delete_dist_table_after_used;

  // heuristic
  Heuristic *heuristic;

  // exploration (MAB) mode
  bool is_exploring;
  std::vector<Arm> arms;
  ThreadPool exploration_thread_pool;

  // scatter (SUO)
  IScatter *scatter;

  // PIBTs for main (non-exploring) search
  std::vector<PIBT *> pibts;

  // for refiner
  int seed_refiner;
  std::list<std::future<Solution>> refiner_pool;

  // for search utils
  std::deque<HNode *> OPEN;
  std::unordered_map<Config, HNode *, ConfigHasher> EXPLORED;
  HNode *H_init;  // start node
  HNode *H_goal;  // goal node

  // for logging
  static std::string MSG;

  int search_iter;
  int time_initial_solution;
  int cost_initial_solution;

  ScatterMABPlanner(const Instance *_ins, int _verbose = 0,
                    const Deadline *_deadline = nullptr, int _seed = 0,
                    int _depth = 0,          // used in recursive LaCAM
                    DistTable *_D = nullptr  // used in recursive LaCAM
  );
  ~ScatterMABPlanner();
  Solution solve();
  void explore_scatters();
  EpochResult run_epoch(int arm_id, IScatter *scatter, std::mt19937 &local_mt);
  bool set_new_config(HNode *S, LNode *M, Config &Q_to);
  HNode *create_highlevel_node(const Config &Q, HNode *parent);
  void rewrite(HNode *H_from, HNode *H_to);
  int get_edge_cost(const Config &C1, const Config &C2);
  Solution backtrack(HNode *H);
  void apply_new_solution(const Solution &plan);
  void set_scatter();
  void set_pibt();
  void set_refiner();
  Solution get_refined_plan(const Solution &plan_origin);
  void logging();
};
