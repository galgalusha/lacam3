#include "../include/pair_wise_h.hpp"
#include"../include/dist_table.hpp"

#include "../include/pair_wise_db.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <unordered_map>

constexpr int NUM_OF_THREADS = 7;
constexpr char* DB_PATH = "/home/galko/dev/mapf_db/";

// --- Anonymous namespace for internal linkage (recommended for globals) ---
namespace {
  // 1. Thread-local storage for the A* closed list (used by worker threads)
  thread_local std::vector<int> t_closed_g;
  thread_local std::vector<uint32_t> t_closed_gen;
  thread_local uint32_t t_current_gen = 0;

  // 2. Shared storage for the BFS reachability analysis (populated by main thread, read by workers)
  std::vector<uint32_t> shared_zero_dh_gen;
  uint32_t shared_zero_current_gen = 0;

  // 3. Zero-allocation queue for the backward BFS (used exclusively by main thread)
  std::vector<std::pair<Vertex*, Vertex*>> shared_bfs_q;
}

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

static DistTable* create_dist_table(Graph* G);

PairWiseHeuristic::PairWiseHeuristic(Graph* _G, std::string _name) : G(_G), name(_name), D(create_dist_table(_G)) {}

std::unordered_set<PairKey> PairWiseHeuristic::get_keys_from_files() {
  std::unordered_set<PairKey> keys;
  if (!std::filesystem::exists(DB_PATH + name)) return keys;
  for (const auto& entry : std::filesystem::directory_iterator(DB_PATH + name)) {
    const std::string name = entry.path().filename().string();
    if (name.rfind("tmp_", 0) == 0) continue; // skip in-progress files
    int lo, hi;
    if (std::sscanf(name.c_str(), "%d_%d.bin", &lo, &hi) == 2)
      keys.insert(to_key((uint16_t)lo, (uint16_t)hi));
  }
  return keys;
}

DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}

int joint_astar(DistTable* D, Vertex* i_start, Vertex* i_goal, Vertex* j_start, Vertex* j_goal) {
  struct State {
    int g, f;
    Vertex* i;
    Vertex* j;
  };

  auto cmp = [](const State& a, const State& b) {
    if (a.f != b.f) return a.f > b.f;
    return a.g < b.g; 
  };
  std::priority_queue<State, std::vector<State>, decltype(cmp)> open(cmp);

  int V = D->K; 
  
  // Initialize the TLS vectors exactly once per thread
  if (t_closed_g.size() < (size_t)V * V) {
    t_closed_g.assign(V * V, INT_MAX);
    t_closed_gen.assign(V * V, 0);
  }
  
  // Increment generation counter, handle overflow to prevent stale data corruption
  if (++t_current_gen == 0) {
    std::fill(t_closed_gen.begin(), t_closed_gen.end(), 0);
    t_current_gen = 1;
  }

  auto encode = [&](int i, int j) { return i * V + j; };

  int h0 = D->get(i_goal->id, i_start->id) + D->get(j_goal->id, j_start->id);
  open.push({0, h0, i_start, j_start});

  while (!open.empty()) {
    auto [g, f, ci, cj] = open.top();
    open.pop();

    if (ci == i_goal && cj == j_goal) return g;

    int ek = encode(ci->id, cj->id);
    
    // Check if state was visited in the current A* generation
    if (t_closed_gen[ek] == t_current_gen && t_closed_g[ek] <= g) continue;
    t_closed_gen[ek] = t_current_gen;
    t_closed_g[ek] = g;

    bool i_at_goal = (ci == i_goal);
    bool j_at_goal = (cj == j_goal);

    int i_deg = ci->neighbor.size();
    int j_deg = cj->neighbor.size();

    // 2. Stack-allocated fixed arrays for extreme L1 cache speed.
    // Assuming max 4-connected grid (4 neighbors) + 1 wait action = size 5.
    Vertex* moves_i[5]; 
    Vertex* moves_j[5];

    // Populate arrays to remove conditionals from the inner loops
    for (int k = 0; k < i_deg; ++k) moves_i[k] = ci->neighbor[k];
    moves_i[i_deg] = ci; 

    for (int k = 0; k < j_deg; ++k) moves_j[k] = cj->neighbor[k];
    moves_j[j_deg] = cj; 

    // 3. Branchless arrays and hoisted heuristics
    for (int i_idx = 0; i_idx <= i_deg; ++i_idx) {
      Vertex* ni = moves_i[i_idx];
      
      int h_i = D->get(i_goal->id, ni->id);
      int step_i = (i_at_goal && ni == i_goal) ? 0 : 1;

      for (int j_idx = 0; j_idx <= j_deg; ++j_idx) {
        Vertex* nj = moves_j[j_idx];

        // Rule out wastes and conflicts
        if (ni == ci && nj == cj && !i_at_goal && !j_at_goal) continue;
        if (ni == nj) continue; // Vertex conflict
        if (ni == cj && nj == ci) continue; // Swap conflict

        int step_j = (j_at_goal && nj == j_goal) ? 0 : 1;
        int ng = g + step_i + step_j;

        int nek = encode(ni->id, nj->id);
        
        // O(1) closed list check bypassing the unordered_map completely
        if (t_closed_gen[nek] == t_current_gen && t_closed_g[nek] <= ng) continue;

        int nf = ng + h_i + D->get(j_goal->id, nj->id);
        open.push({ng, nf, ni, nj});
      }
    }
  }

  return INT_MAX;
}

bool PairWiseHeuristic::can_interfere(DistTable* D, int i_start, int i_goal, int j_start, int j_goal) {
  // D->get(target, source) gives distance from source to target

  // 1. Pairwise start/goal distances
  int dist_s_ij  = D->get(j_start, i_start); // dist from i_start to j_start
  int dist_g_ij  = D->get(j_goal,  i_goal);  // dist from i_goal  to j_goal

  // 2. Individual shortest path lengths
  int dist_sg_ii = D->get(i_goal,  i_start); // dist from i_start to i_goal
  int dist_sg_jj = D->get(j_goal,  j_start); // dist from j_start to j_goal

  // 3. Evaluate the Psi constraint
  int psi_ij = dist_s_ij + dist_g_ij - dist_sg_ii - dist_sg_jj;
  if (psi_ij > 0) {
    return false;
  }

  // 4. Cross-distances for the Lambda constraint
  int dist_sg_ji = D->get(i_goal,  j_start); // dist from j_start to i_goal
  int dist_sg_ij = D->get(j_goal,  i_start); // dist from i_start to j_goal

  // 5. Evaluate the Lambda constraint
  int lambda_ij = dist_sg_ii - dist_sg_ji;
  int lambda_ji = dist_sg_jj - dist_sg_ij;

  if (lambda_ij + lambda_ji < 0) {
    return false;
  }

  return true;
}

bool PairWiseHeuristic::has_alternative_path(DistTable* D, Vertex* blocked, Vertex* v_s, Vertex* v_g) {
  int target_dist = D->get(v_g->id, v_s->id);
  if (target_dist == D->K) return false;
  if (v_s == blocked) return false;

  // BFS over shortest-path DAG; visited prevents exponential re-expansion on open grids
  std::vector<bool> visited(D->K, false);
  std::vector<Vertex*> frontier;
  frontier.push_back(v_s);
  visited[v_s->id] = true;

  while (!frontier.empty()) {
    std::vector<Vertex*> next;
    for (Vertex* cur : frontier) {
      if (cur == v_g) return true;
      int remaining = D->get(v_g->id, cur->id);
      for (Vertex* nb : cur->neighbor) {
        if (nb == blocked) continue;
        if (visited[nb->id]) continue;
        if (D->get(v_g->id, nb->id) != remaining - 1) continue;
        visited[nb->id] = true;
        next.push_back(nb);
      }
    }
    frontier = std::move(next);
  }
  return false;
}

void populate_zero_dh_by_bfs(DistTable* D, Vertex* i_goal, Vertex* j_goal) {
  int V = D->K; 

  // Handle generation overflow
  if (++shared_zero_current_gen == 0) {
    std::fill(shared_zero_dh_gen.begin(), shared_zero_dh_gen.end(), 0);
    shared_zero_current_gen = 1;
  }

  // Reset the flat queue without releasing memory capacity
  shared_bfs_q.clear(); 
  shared_bfs_q.push_back({i_goal, j_goal});
  
  // Mark initial state
  shared_zero_dh_gen[i_goal->id * V + j_goal->id] = shared_zero_current_gen;

  size_t head = 0;
  while (head < shared_bfs_q.size()) {
    auto [u, v] = shared_bfs_q[head++];

    int d_i = D->get(i_goal->id, u->id);
    int d_j = D->get(j_goal->id, v->id);

    // 1. Build valid moves for agent i
    Vertex* moves_i[5];
    int deg_i = 0;
    for (Vertex* n : u->neighbor) {
      // Only keep neighbors that step strictly away from the goal
      if (D->get(i_goal->id, n->id) == d_i + 1) {
        moves_i[deg_i++] = n;
      }
    }
    // Conditionally append wait action
    if (u == i_goal) {
      moves_i[deg_i++] = u; 
    }

    // 2. Build valid moves for agent j
    Vertex* moves_j[5];
    int deg_j = 0;
    for (Vertex* n : v->neighbor) {
      // Only keep neighbors that step strictly away from the goal
      if (D->get(j_goal->id, n->id) == d_j + 1) {
        moves_j[deg_j++] = n;
      }
    }
    // Conditionally append wait action
    if (v == j_goal) {
      moves_j[deg_j++] = v;
    }

    // 3. Clean, branchless nested loops over pre-filtered moves
    for (int i_idx = 0; i_idx < deg_i; ++i_idx) {
      Vertex* u_prime = moves_i[i_idx];

      for (int j_idx = 0; j_idx < deg_j; ++j_idx) {
        Vertex* v_prime = moves_j[j_idx];

        // Skip pure joint-wait to avoid infinite loop at the initial (goal, goal) state
        if (u_prime == u && v_prime == v) continue;
        
        // Check conflicts
        if (u_prime == v_prime) continue; // Vertex conflict
        if (u_prime == v && v_prime == u) continue; // Swap conflict

        int ek = u_prime->id * V + v_prime->id;
        
        // If not visited in the current generation, mark and push
        if (shared_zero_dh_gen[ek] != shared_zero_current_gen) {
          shared_zero_dh_gen[ek] = shared_zero_current_gen;
          shared_bfs_q.push_back({u_prime, v_prime});
        }
      }
    }
  }
}

void PairWiseHeuristic::construct() { 
  const int num_vertices = G->V.size();

  long long count_evaluated = 0;
  long long count_interfering = 0;
  shared_zero_dh_gen = std::vector<uint32_t>(num_vertices * num_vertices, 0);

  // 2. Iterate over all valid 4-tuples (i_start, i_goal, j_start, j_goal)
  const auto done_keys = get_keys_from_files();
  long long total_outer = (long long)num_vertices * (num_vertices - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);
  for (int i_g = 0; i_g < num_vertices; ++i_g) {
    for (int j_g = i_g + 1; j_g < num_vertices; ++j_g, ++outer_idx) {
      if (done_keys.count(to_key((uint16_t)i_g, (uint16_t)j_g))) continue;
      int pct = (int)(outer_idx * 100000 / total_outer); // units of 0.001%
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000; // 50-char bar over 100000 units
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }

      populate_zero_dh_by_bfs(D, G->V[i_g], G->V[j_g]);

      std::string tmp_path = DB_PATH + name + "/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); } // ensure file exists even if empty

      std::vector<std::future<ThreadResult>> futures;

      for (int i_s = 0; i_s < num_vertices; ++i_s) {
        if (i_s == i_g) continue;

        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          for (int j_s = 0; j_s < (int)G_ref.V.size(); ++j_s) {
            if (j_s == j_g) continue;
            if (i_s == j_s) continue;
            ++result.evaluated;

            int ek = i_s * D->K + j_s;
            if (shared_zero_dh_gen[ek] == shared_zero_current_gen) continue;

            if (can_interfere(D_ptr, i_s, i_g, j_s, j_g)) {
              Vertex* vi_s = G_ref.V[i_s];
              Vertex* vi_g = G_ref.V[i_g];
              Vertex* vj_s = G_ref.V[j_s];
              Vertex* vj_g = G_ref.V[j_g];

              int independent_cost = D_ptr->get(i_g, i_s) + D_ptr->get(j_g, j_s);
              int joint_cost = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g);
              int d_h = joint_cost - independent_cost;
              if (d_h > 0)
                result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)d_h});
            }
          }
          return result;
        }));
      }

      // Main thread collects results in submission order and flushes each batch
      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        count_evaluated += evaluated;
        count_interfering += entries.size();
        append_entries(tmp_path, entries);
      }

      // Promote tmp file to permanent
      std::filesystem::rename(tmp_path, final_path);
    }
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "Total 4-tuples evaluated: " << count_evaluated << std::endl;
  std::cout << "Pairs flagged for A*:     " << count_interfering << std::endl;
}

void PairWiseHeuristic::construct_for_instance(const Config& goals) {

  long long count_evaluated = 0;
  long long count_interfering = 0;
  const int num_vertices = G->V.size();
  const int K = (int)goals.size();
  shared_zero_dh_gen = std::vector<uint32_t>(num_vertices * num_vertices, 0);

  const auto done_keys = get_keys_from_files();
  long long total_outer = (long long)K * (K - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);
  for (int agent1 = 0; agent1 < K; ++agent1) {
    for (int agent2 = agent1 + 1; agent2 < K; ++agent2, ++outer_idx) {
      int i_g = std::min(goals[agent1]->id, goals[agent2]->id);
      int j_g = std::max(goals[agent1]->id, goals[agent2]->id);
      if (done_keys.count(to_key((uint16_t)i_g, (uint16_t)j_g))) continue;

      int pct = (int)(outer_idx * 100000 / total_outer); // units of 0.001%
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000; // 50-char bar over 100000 units
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }

      populate_zero_dh_by_bfs(D, G->V[i_g], G->V[j_g]);

      std::string tmp_path = DB_PATH + name + "/tmp_" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); } // ensure file exists even if empty

      std::vector<std::future<ThreadResult>> futures;

      for (int i_s = 0; i_s < num_vertices; ++i_s) {
        if (i_s == i_g) continue;

        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          for (int j_s = 0; j_s < (int)G_ref.V.size(); ++j_s) {
            if (j_s == j_g) continue;
            if (i_s == j_s) continue;
            ++result.evaluated;

            int ek = i_s * D->K + j_s;
            
            if (shared_zero_dh_gen[ek] == shared_zero_current_gen) continue;

            if (can_interfere(D_ptr, i_s, i_g, j_s, j_g)) {
              Vertex* vi_s = G_ref.V[i_s];
              Vertex* vi_g = G_ref.V[i_g];
              Vertex* vj_s = G_ref.V[j_s];
              Vertex* vj_g = G_ref.V[j_g];

              int independent_cost = D_ptr->get(i_g, i_s) + D_ptr->get(j_g, j_s);
              int joint_cost = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g);
              int d_h = joint_cost - independent_cost;
              if (d_h > 0)
                result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)d_h});
            }
          }
          return result;
        }));
      }

      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        count_evaluated += evaluated;
        count_interfering += entries.size();
        append_entries(tmp_path, entries);
      }

      std::filesystem::rename(tmp_path, final_path);
    }
  }

  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
  std::cout << "Total 4-tuples evaluated: " << count_evaluated << std::endl;
  std::cout << "Pairs flagged for A*:     " << count_interfering << std::endl;
}

void PairWiseHeuristic::construct_for_instance_only_goals(const Config& goals) {

  const int K = (int)goals.size();

  long long total_outer = (long long)K * (K - 1) / 2;
  long long outer_idx = 0;
  int last_pct = -1;
  ThreadPool pool(NUM_OF_THREADS);

  // absl::flat_hash_map<PairKey, absl::flat_hash_map<PairKey, uint16_t>> pair_data2;

  for (int agent1 = 0; agent1 < K; ++agent1) {
    for (int agent2 = agent1 + 1; agent2 < K; ++agent2, ++outer_idx) {
      int pct = (int)(outer_idx * 100000 / total_outer);
      if (pct != last_pct) {
        last_pct = pct;
        int filled = pct / 2000;
        std::cout << "\r[" << std::string(filled, '#') << std::string(50 - filled, ' ')
                  << "] " << (pct / 1000) << "." << std::setw(3) << std::setfill('0') << (pct % 1000) << "%" << std::flush;
      }
      int i_g = std::min(goals[agent1]->id, goals[agent2]->id);
      int j_g = std::max(goals[agent1]->id, goals[agent2]->id);

      std::string tmp_path   = DB_PATH + name + "/tmp_" + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::string final_path = DB_PATH + name + "/"     + std::to_string(i_g) + "_" + std::to_string(j_g) + ".bin";
      std::filesystem::create_directories(DB_PATH + name);
      { std::ofstream touch(tmp_path, std::ios::binary); }

      std::vector<std::future<ThreadResult>> futures;

      // case 1: i_s == i_g, one task per j_s
      for (int j_s = 0; j_s < (int)G->V.size(); ++j_s) {
        if (j_s == j_g || j_s == i_g) continue;
        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          int i_s = i_g;
          if (D_ptr->get(j_s, i_g) + D_ptr->get(i_g, j_g) != D_ptr->get(j_s, j_g)) return result;
          Vertex* vi_s = G_ref.V[i_s];
          Vertex* vi_g = G_ref.V[i_g];
          Vertex* vj_s = G_ref.V[j_s];
          Vertex* vj_g = G_ref.V[j_g];
          if (has_alternative_path(D_ptr, vi_g, vj_s, vj_g)) return result;
          int dh = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g) - D_ptr->get(j_s, j_g) - D_ptr->get(i_s, i_g);
          if (dh != 0)
            result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)dh});
          return result;
        }));
      }

      // case 2: j_s == j_g, one task per i_s
      for (int i_s = 0; i_s < (int)G->V.size(); ++i_s) {
        if (i_s == i_g || i_s == j_g) continue;
        futures.push_back(pool.submit([=, D_ptr = D, &G_ref = *G]() {
          ThreadResult result{{}, 0};
          int j_s = j_g;
          if (D_ptr->get(i_s, j_g) + D_ptr->get(j_g, i_g) != D_ptr->get(i_s, i_g)) return result;
          Vertex* vi_s = G_ref.V[i_s];
          Vertex* vi_g = G_ref.V[i_g];
          Vertex* vj_s = G_ref.V[j_s];
          Vertex* vj_g = G_ref.V[j_g];
          if (has_alternative_path(D_ptr, vj_g, vi_s, vi_g)) return result;
          int dh = joint_astar(D_ptr, vi_s, vi_g, vj_s, vj_g) - D_ptr->get(j_s, j_g) - D_ptr->get(i_s, i_g);
          if (dh != 0)
            result.entries.push_back({(uint16_t)i_s, (uint16_t)j_s, (uint16_t)dh});
          return result;
        }));
      }

      std::vector<PairEntry> all_entries;
      for (auto& fut : futures) {
        auto [entries, evaluated] = fut.get();
        all_entries.insert(all_entries.end(), entries.begin(), entries.end());
      }
      std::sort(all_entries.begin(), all_entries.end(), [](const PairEntry& a, const PairEntry& b) {
        if (a.i_start != b.i_start) return a.i_start < b.i_start;
        return a.j_start < b.j_start;
      });
      append_entries(tmp_path, all_entries);
      std::filesystem::rename(tmp_path, final_path);
    }
  }
  std::cout << "\r[" << std::string(50, '#') << "] 100%" << std::endl;
}

void PairWiseHeuristic::load_bin_file(uint16_t lo, uint16_t hi, const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return;
  PairEntry entry;
  while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
    uint8_t dh;
    if (entry.dh > 255) {
      std::cout << "[pair_wise_h] warning: dh value " << entry.dh << " exceeds 255, trimming to 255" << std::endl;
      dh = 255;
    } else {
      dh = (uint8_t)entry.dh;
    }
    JointKey jk = to_joint_key(lo, hi, entry.i_start, entry.j_start);
    uint8_t idx = (uint8_t)(jk >> 32);
    uint32_t mk = (uint32_t)(jk & 0xFFFFFFFF);
    pair_data[idx][mk] = dh;
  }
}

void PairWiseHeuristic::load_all(Instance* ins) {
  int N = (int)ins->goals.size();
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      uint16_t gi = (uint16_t)ins->goals[i]->id;
      uint16_t gj = (uint16_t)ins->goals[j]->id;
      uint16_t lo = std::min(gi, gj), hi = std::max(gi, gj);
      std::string path = DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin";
      load_bin_file(lo, hi, path);
    }
  }
}

std::unordered_set<int> PairWiseHeuristic::bfs_with_margin(Vertex* start, Vertex* goal, int margin) const {
  const int optimal = D->get(goal->id, start->id);
  std::unordered_set<int> explored;
  std::queue<Vertex*> open;
  explored.insert(start->id);
  open.push(start);
  while (!open.empty()) {
    Vertex* v = open.front(); open.pop();
    for (Vertex* nb : v->neighbor) {
      if (explored.count(nb->id)) continue;
      if (D->get(goal->id, nb->id) <= optimal + margin) {
        explored.insert(nb->id);
        open.push(nb);
      }
    }
  }
  return explored;
}

void PairWiseHeuristic::load_bin_file_filtered(uint16_t lo, uint16_t hi, const std::string& path,
                                                bool nearest_is_lo, const std::unordered_set<int>& vertex_set) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return;
  PairEntry entry;
  while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
    // only load entries where nearest_agent's start is within the margin set
    int nearest_start = nearest_is_lo ? entry.i_start : entry.j_start;
    if (!vertex_set.count(nearest_start)) continue;
    uint8_t dh;
    if (entry.dh > 255) {
      dh = 255;
    } else {
      dh = (uint8_t)entry.dh;
    }
    JointKey jk = to_joint_key(lo, hi, entry.i_start, entry.j_start);
    uint8_t idx = (uint8_t)(jk >> 32);
    uint32_t mk = (uint32_t)(jk & 0xFFFFFFFF);
    pair_data[idx][mk] = dh;
  }
}

void PairWiseHeuristic::load_some(Instance* ins, int margin) {
  const int N = (int)ins->goals.size();
  const int total = N * (N - 1) / 2;
  int done = 0;
  const int bar_width = 40;
  const auto print_bar = [&]() {
    float frac = total > 0 ? (float)done / total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << done << "/" << total << std::flush;
  };
  print_bar();
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      const int dist_i = D->get(ins->goals[i]->id, ins->starts[i]->id);
      const int dist_j = D->get(ins->goals[j]->id, ins->starts[j]->id);
      const int nearest = (dist_i <= dist_j) ? i : j;

      const uint16_t gi = (uint16_t)ins->goals[i]->id;
      const uint16_t gj = (uint16_t)ins->goals[j]->id;
      const uint16_t lo = std::min(gi, gj), hi = std::max(gi, gj);
      // nearest_is_lo: nearest agent's goal is the smaller of the two (lo side in DB)
      const bool nearest_is_lo = (ins->goals[nearest]->id == lo);

      const auto vertex_set = bfs_with_margin(ins->starts[nearest], ins->goals[nearest], margin);
      const std::string path = DB_PATH + name + "/" + std::to_string(lo) + "_" + std::to_string(hi) + ".bin";
      load_bin_file_filtered(lo, hi, path, nearest_is_lo, vertex_set);
      ++done;
      print_bar();
    }
  }
  std::cout << std::endl;
}

uint8_t PairWiseHeuristic::get(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) const {
  bool flipped = i_goal > j_goal;
  uint16_t lo  = flipped ? j_goal  : i_goal;
  uint16_t hi  = flipped ? i_goal  : j_goal;
  uint16_t is  = flipped ? j_start : i_start;
  uint16_t js  = flipped ? i_start : j_start;
  JointKey jk = to_joint_key(lo, hi, is, js);
  uint8_t idx = (uint8_t)(jk >> 32);
  uint32_t mk = (uint32_t)(jk & 0xFFFFFFFF);
  auto it = pair_data[idx].find(mk);
  return it != pair_data[idx].end() ? it->second : 0;
}

void PairWiseHeuristic::integration_test() {
  std::string dir = DB_PATH + name;
  if (!std::filesystem::exists(dir)) {
    std::cout << "[integration_test] directory not found: " << dir << std::endl;
    return;
  }

  std::vector<std::filesystem::path> files;
  for (const auto& e : std::filesystem::directory_iterator(dir)) {
    const std::string fname = e.path().filename().string();
    if (fname.rfind("tmp_", 0) == 0) continue;
    int lo, hi;
    if (std::sscanf(fname.c_str(), "%d_%d.bin", &lo, &hi) == 2)
      files.push_back(e.path());
  }

  std::mt19937 rng(42);
  if ((int)files.size() > 100) {
    std::shuffle(files.begin(), files.end(), rng);
    files.resize(100);
  }

  long long assert_pass = 0, assert_fail = 0;
  // index 1-10 for dh=1..10, index 11 for dh>10 (index 0 unused)
  std::array<long long, 12> dh_counts{};
  std::vector<std::string> failures;

  for (const auto& fpath : files) {
    std::string fname = fpath.filename().string();
    int lo_i = 0, hi_i = 0;
    std::sscanf(fname.c_str(), "%d_%d.bin", &lo_i, &hi_i);
    uint16_t lo = (uint16_t)lo_i, hi = (uint16_t)hi_i;

    load_bin_file(lo, hi, fpath.string());

    std::ifstream f(fpath.string(), std::ios::binary);
    PairEntry entry;
    while (f.read(reinterpret_cast<char*>(&entry), sizeof(PairEntry))) {
      uint8_t expected = entry.dh > 255 ? 255 : (uint8_t)entry.dh;
//      uint8_t actual = get(lo, hi, entry.i_start, entry.j_start);
      uint8_t actual = get(hi, lo, entry.j_start, entry.i_start);

      int bucket = expected <= 10 ? (int)expected : 11;
      dh_counts[bucket]++;

      if (actual == expected) {
        ++assert_pass;
      } else {
        ++assert_fail;
        if (failures.size() < 20)
          failures.push_back("file=" + fname + " start=(" + std::to_string(entry.i_start) +
                             "," + std::to_string(entry.j_start) + ") expected=" +
                             std::to_string(expected) + " got=" + std::to_string(actual));
      }
    }

    for (auto& m : pair_data) m.clear();
  }

  std::cout << "[integration_test] Results over " << files.size() << " file(s):" << std::endl;
  std::cout << "  Assertions passed: " << assert_pass << std::endl;
  std::cout << "  Assertions failed: " << assert_fail << std::endl;
  std::cout << "  dh distribution:" << std::endl;
  for (int d = 1; d <= 10; ++d)
    std::cout << "    dh=" << d << ": " << dh_counts[d] << std::endl;
  std::cout << "    dh>10: " << dh_counts[11] << std::endl;
  if (!failures.empty()) {
    std::cout << "  Failed assertions (first " << failures.size() << "):" << std::endl;
    for (const auto& msg : failures) std::cout << "    " << msg << std::endl;
  }
}

void PairWiseHeuristic::test() {

  std::vector<std::string> grid = {
    "....",
    "....",
    "@@@.",
    "..@.",
    "....",
  };
  Graph* G = new Graph(grid);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  auto A_start = coord(0, 0);
  auto A_goal  = coord(4, 1);

  auto B_start = coord(1, 1);
  auto B_goal  = coord(3, 0);

  // auto C_start = coord(3, 1);
  // auto C_goal  = coord(0, 3);

  auto C_start = coord(4, 2);
  auto C_goal  = coord(4, 2);

  Config starts = { A_start, B_start, C_start };
  Config goals  = { A_goal,  B_goal,  C_goal  };

  std::string test2 = "test2";
  std::string test2_goals = "test2_goals";
  PairWiseHeuristic pwh_no_goals(G, test2);
  pwh_no_goals.construct_for_instance(goals);
  PairWiseHeuristic pwh_goals(G, test2_goals);
  pwh_goals.construct_for_instance_only_goals(goals);
  merge_goal_folder(DB_PATH + test2, DB_PATH + test2_goals);

  PairWiseHeuristic pwh(G, test2);
  Instance* ins = new Instance(G, starts, goals, starts.size());
  pwh.load_all(ins);

  std::cout << "A with B: " << (int)pwh.get(A_goal->id, B_goal->id, A_start->id, B_start->id) << std::endl;
  std::cout << "A with C: " << (int)pwh.get(A_goal->id, C_goal->id, A_start->id, C_start->id) << std::endl;
  std::cout << "B with C: " << (int)pwh.get(B_goal->id, C_goal->id, B_start->id, C_start->id) << std::endl;
  std::cout << "Done 2" << std::endl;
}
