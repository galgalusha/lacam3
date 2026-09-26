#include "../include/horizon_pair_db.hpp"
#include "../include/thread_pool.hpp"
#include "../include/pair_wise_bin.hpp" // for thread pool

#include <atomic>
#include <iomanip>
#include <mutex>
#include <algorithm>
#include <iostream>
#include <absl/container/flat_hash_map.h>

/**
 * ## Thoughrs For PIBT
 * For an agent i, we need to find every agent j that has a conflict within the horizon.
 * Suppose horizon=5. The MDD of agent i contains thousands of possible conflicts in
 * the DB. Iterating them is not a good option. Iterating through all agents j for every
 * agent i is also a bad option. We don't want N^2 complexity in PIBT.
 * What we do is pre-calculate (before the solver begins) an MDD for each possible agent 
 * setting (v_i, goal_i). For horizon=5, the max MDD size is 21 vertices.
 * During PIBT, before iterating the agents and calling funcPIBT, iterate through all
 * agent MDDs to build an index from (vertex, time) to an agent id. Complexity is N*21.
 * We don't use a real map but rather an array with an index of (time*|V|+vertex.id).
 * When agent i considers moving from v_i to u_i, we can pick the MDD for u_i from the
 * MDD cache. For each (time, vertex) in this MDD, we can find conflicts using the index.
 * However, this assumes the other agents are stationary. So when an agent actually moves,
 * we need to remove its previous MDD from the index and update the index with its new MDD.
 * Another option is not to create accurate MDDs in the pre-calculation but rather
 * an extended structure, lets call it EMDD, that considers all possible moves.
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

  // Pre-allocate master storage
  mdd_id_by_agent.assign(V_SIZE * V_SIZE, 0);
  mdd_by_id.resize(V_SIZE * V_SIZE);
  mdd_by_t_s.assign(V_SIZE * (HORIZON + 1), std::vector<uint32_t>());

  std::mutex mdd_count_mtx; // Only used to safely increment the global ID
  std::mutex mdd_mtx2;      // Protects push_back on mdd_by_t_s
  std::mutex io_mtx;        // Protects console output

  const long long total = (long long)V_SIZE * (long long)V_SIZE;
  std::atomic done = 0;
  const int bar_width = 40;

  auto print_bar = [&]() {
    std::lock_guard lock(io_mtx);
    float frac = total > 0 ? (float)done.load() / (float)total : 1.0f;
    int filled = (int)(frac * bar_width);
    std::cout << "\r[";
    for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
  };

  print_bar();

  // Task processes an ENTIRE start vertex (v_i)
  auto task = [&](int v_i) {
    // Completely thread-local map! No mutex needed to access this.
    absl::flat_hash_map<MDD, uint32_t> local_mdd_to_id;
    
    for (int g_i = 0; g_i < (int)V_SIZE; g_i++) {
      int agent_id = v_i * (int)V_SIZE + g_i;
      
      MDD mdd; 
      mdd.populate(D, G->V[v_i], G->V[g_i], HORIZON);
      
      uint32_t mdd_id;
      bool is_new_mdd = false;

      auto it = local_mdd_to_id.find(mdd);
      if (it != local_mdd_to_id.end()) {
        mdd_id = it->second;
      } else {
        // 1. Get a globally unique ID
        {
          std::lock_guard lock(mdd_count_mtx);
          mdd_id = mdd_count++;
        }
        is_new_mdd = true;
        
        // 2. Lock-free write! mdd_by_id is pre-allocated and mdd_id is strictly unique.
        mdd_by_id[mdd_id] = mdd;
      }

      // Lock-free write! agent_id is strictly unique per loop iteration.
      mdd_id_by_agent[agent_id] = mdd_id;

      if (is_new_mdd) {
        // Protect mdd_by_t_s because different v_i threads might hit the same space-time vertex
        std::lock_guard lock(mdd_mtx2);
        for (int t = 0; t < (int)mdd.frontiers.size(); t++) {
          for (Vertex* v : mdd.frontiers[t]) {
            int idx = t * (int)V_SIZE + v->id;
            mdd_by_t_s[idx].push_back(mdd_id); 
          }
        }
      }

      // 3. Emplace into the local map AFTER iterating frontiers so we don't move an empty MDD
      if (is_new_mdd) {
        local_mdd_to_id.emplace(std::move(mdd), mdd_id);
      }

      ++done;
    }
    
    // Update progress bar once per start vertex instead of every inner loop
    print_bar(); 
  };

  ThreadPool pool(NUM_OF_THREADS);
  std::vector<std::future<ThreadResult>> futures;
  futures.reserve(V_SIZE);

  // Submit ONE task per start vertex
  for (int v_i = 0; v_i < (int)V_SIZE; v_i++) {
    futures.push_back(pool.submit([&task, v_i]() { task(v_i); return ThreadResult{}; }));
  }

  for (auto& fut : futures) fut.get();

  print_bar(); // Final 100% update
  std::cout << "\nNum of agents: " << V_SIZE * V_SIZE << std::endl;
  std::cout << "Num of MDDs  : " << mdd_count << std::endl;
}



void HorizonPairDB::generate_conflicting_pairs() {
    // size_t V_SIZE = G->V.size();
    // size_t N = V_SIZE * V_SIZE; // Total number of possible agents
    
    // // The Generation Array: heavily cache-optimized O(1) deduplication.
    // // seen_by[b] = a means "Agent b has already been recorded as conflicting with Agent a"
    // // This entirely eliminates the need to clear arrays or use std::find/std::unordered_set.
    // std::vector<int> seen_by(N, -1); 
    
    // uint64_t total_unique_pairs = 0;

    // const int bar_width = 40;
    // auto print_bar = [&](size_t done) {
    //     float frac = N > 0 ? (float)done / (float)N : 1.0f;
    //     int filled = (int)(frac * bar_width);
    //     std::cout << "\r[";
    //     for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
    //     std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%"
    //               << " pairs: " << total_unique_pairs << std::flush;
    // };

    // // Iterate agent by agent
    // for (int a = 0; a < N; ++a) {
    //     print_bar(a);
    //     MDD* mdd_a = &mdd_id_by_agent[a];
        
    //     // Skip if agent 'a' has no valid MDD 
    //     if (mdd_a->frontiers.empty()) continue; 

    //     // Trace Agent A's path through space-time
    //     for (int t = 0; t < mdd_a->frontiers.size(); ++t) {
    //         for (Vertex* v : mdd_a->frontiers[t]) {
    //             int idx = t * V_SIZE + v->id;
                
    //             // Look at all other agents sharing this exact (v, t)
    //             for (MDD* mdd_b : mdd_by_t_s[idx]) {
    //                 int b = mdd_b->agent_id;
                    
    //                 // Decode IDs back to graph vertices
    //                 int v_a = a / V_SIZE;
    //                 int g_a = a % V_SIZE;
    //                 int v_b = b / V_SIZE;
    //                 int g_b = b % V_SIZE;

    //                 // Reject physically impossible MAPF states
    //                 if (v_a == v_b || g_a == g_b) continue;

    //                 // Strict ordering (b > a) prevents counting both (A,B) and (B,A).
    //                 // seen_by[b] != a prevents counting (A,B) twice if they 
    //                 // collide at multiple different time steps.
    //                 if (b > a && seen_by[b] != a) {
    //                     seen_by[b] = a; 
    //                     total_unique_pairs++;
    //                 }
    //             }
    //         }
    //     }
    // }

    // std::cout << std::endl;
    // std::cout << "Unique conflicting pairs: " << total_unique_pairs << "\n";
}

