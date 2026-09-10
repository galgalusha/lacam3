#include "../include/horizon_pair_db.hpp"
#include "../include/thread_pool.hpp"
#include "../include/pair_wise_bin.hpp" // for thread pool

#include <atomic>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <iostream>

/**
 * TODO:
 * 1. Group agents by MDD
 * 2. Use Joint MDD to filter real conflicts
 * 3. To distinct between dh=1 (wait is required) and dh>=2 (detour is required),
 *    it is enough to check if a shifted MDD makes the joint MDD stop detecting conflicts.
 */

const int HorizonPairDB::HORIZON = 5;

static const int NUM_OF_THREADS = 8;


static DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}

HorizonPairDB::HorizonPairDB(Graph* _G) : G(_G), D(create_dist_table(G)) {}


void HorizonPairDB::generate_mdds() {
  size_t V_SIZE = G->V.size();

  // 1. Pre-allocate master storage to prevent reallocation and keep MDDs contiguous in cache
  all_mdds.reserve(V_SIZE * V_SIZE);

  // 2. Fix the out-of-bounds crash: HORIZON + 1 time steps. 
  // Each bucket now holds a vector of MDD pointers.
  idx_mdds.assign(V_SIZE * (HORIZON + 1), std::vector<MDD*>());

  std::mutex all_mdds_mtx;  // guards all_mdds.emplace_back
  std::mutex idx_mdds_mtx;  // guards idx_mdds bucket pushes
  std::mutex io_mtx;        // guards progress bar output

  const long long total = (long long)V_SIZE * (long long)V_SIZE;
  std::atomic<long long> done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    std::lock_guard<std::mutex> lock(io_mtx);
    float frac = total > 0 ? (float)done.load() / (float)total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  // Task: build and populate one MDD for the (v_i, g_i) pair, then index it.
  auto task = [&](int v_i, int g_i) -> ThreadResult {
    int agent_id = v_i * (int)V_SIZE + g_i;

    MDD* mdd;
    {
      // 3. Construct in-place. No 'new' keywords, no memory leaks.
      std::lock_guard<std::mutex> lock(all_mdds_mtx);
      all_mdds.emplace_back(agent_id);
      mdd = &all_mdds.back();
    }

    mdd->populate(D, G->V[v_i], G->V[g_i], HORIZON);

    {
      std::lock_guard<std::mutex> lock(idx_mdds_mtx);
      for (int t = 0; t < (int)mdd->frontiers.size(); t++) {
        for (Vertex* v : mdd->frontiers[t]) {
          int idx = t * (int)V_SIZE + v->id;
          // 4. Push to the bucket. Now we capture ALL agents sharing this space-time coordinate.
          idx_mdds[idx].push_back(mdd);
        }
      }
    }

    ++done;
    print_bar();
    return ThreadResult{};
  };

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(V_SIZE * V_SIZE);

  for (int v_i = 0; v_i < (int)V_SIZE; v_i++) {
    for (int g_i = 0; g_i < (int)V_SIZE; g_i++) {
      futures.push_back(pool.submit([&task, v_i, g_i]() { return task(v_i, g_i); }));
    }
  }

  for (auto& fut : futures) fut.get();

  std::cout << std::endl;
}



void HorizonPairDB::generate_conflicting_pairs() {
    size_t V_SIZE = G->V.size();
    size_t N = V_SIZE * V_SIZE; // Total number of possible agents
    
    // The Generation Array: heavily cache-optimized O(1) deduplication.
    // seen_by[b] = a means "Agent b has already been recorded as conflicting with Agent a"
    // This entirely eliminates the need to clear arrays or use std::find/std::unordered_set.
    std::vector<int> seen_by(N, -1); 
    
    uint64_t total_unique_pairs = 0;

    const int bar_width = 40;
    auto print_bar = [&](size_t done) {
        float frac = N > 0 ? (float)done / (float)N : 1.0f;
        int filled = (int)(frac * bar_width);
        std::cout << "\r[";
        for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
        std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%"
                  << " pairs: " << total_unique_pairs << std::flush;
    };

    // Iterate agent by agent
    for (int a = 0; a < N; ++a) {
        print_bar(a);
        MDD* mdd_a = &all_mdds[a];
        
        // Skip if agent 'a' has no valid MDD 
        if (mdd_a->frontiers.empty()) continue; 

        // Trace Agent A's path through space-time
        for (int t = 0; t < mdd_a->frontiers.size(); ++t) {
            for (Vertex* v : mdd_a->frontiers[t]) {
                int idx = t * V_SIZE + v->id;
                
                // Look at all other agents sharing this exact (v, t)
                for (MDD* mdd_b : idx_mdds[idx]) {
                    int b = mdd_b->agent_id;
                    
                    // Decode IDs back to graph vertices
                    int v_a = a / V_SIZE;
                    int g_a = a % V_SIZE;
                    int v_b = b / V_SIZE;
                    int g_b = b % V_SIZE;

                    // Reject physically impossible MAPF states
                    if (v_a == v_b || g_a == g_b) continue;

                    // Strict ordering (b > a) prevents counting both (A,B) and (B,A).
                    // seen_by[b] != a prevents counting (A,B) twice if they 
                    // collide at multiple different time steps.
                    if (b > a && seen_by[b] != a) {
                        seen_by[b] = a; 
                        total_unique_pairs++;
                    }
                }
            }
        }
    }

    std::cout << std::endl;
    std::cout << "Unique conflicting pairs: " << total_unique_pairs << "\n";
}

