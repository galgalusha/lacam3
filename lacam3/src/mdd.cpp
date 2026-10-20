#include "../include/mdd.hpp"
#include "../include/drawing.hpp"

#include "absl/container/inlined_vector.h"
#include <sstream>
#include <string>
#include <unordered_map>


void MDD::populate(DistTable* D, Vertex* v_i, Vertex* v_g, int size, int M) {
  frontiers.resize(size);
  frontiers[0].push_back(v_i);
  
  int D_0 = D->get(v_i->id, v_g->id);

  int t;
  for (t = 1; t < size; t++) { 
    for (Vertex* v_t_minus_1 : frontiers[t - 1]) {
      for (Vertex* v_t : v_t_minus_1->neighbor) {
        if (D->get(v_t->id, v_g->id) <= D_0 + M - t) {
          if (std::find(frontiers[t].begin(), frontiers[t].end(), v_t) == frontiers[t].end()) {
            frontiers[t].push_back(v_t);
          }
        }
      }
      // add wait
      if (M > 0 && D->get(v_t_minus_1->id, v_g->id) <= D_0 + M - t) {
        if (std::find(frontiers[t].begin(), frontiers[t].end(), v_t_minus_1) == frontiers[t].end()) {
          frontiers[t].push_back(v_t_minus_1);
        }        
      }
    }
    if (frontiers[t].empty()) break; 
  }
  
  for (int t_pad = t; t_pad < size; t_pad++) frontiers[t_pad].push_back(v_g);
}


std::string MDD::str() const {
  std::ostringstream oss;
  oss << "{ ";
  for (size_t t = 0; t < frontiers.size(); t++) {
    if (t > 0) oss << ", ";
    oss << t << ": [";
    for (size_t i = 0; i < frontiers[t].size(); i++) {
      if (i > 0) oss << ", ";
      oss << frontiers[t][i]->id;
    }
    oss << "]";
  }
  oss << " }";
  return oss.str();
}

void MDD::render(Instance* ins) {
  int agent_id = frontiers[0][0]->id; // not really an agent id, but its OK for coloring
  using namespace drawing_detail;
  const Graph* G = ins->G;
  const int W = G->width;
  const int H = G->height;

  // map each vertex index (U-index) to the smallest depth t at which it appears
  std::unordered_map<int, int> depth_of;
  for (int t = 0; t < (int)frontiers.size(); t++) {
    for (Vertex* v : frontiers[t]) {
      auto [it, inserted] = depth_of.emplace(v->index, t);
      (void)it;
      (void)inserted;
    }
  }

  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      int idx = W * y + x;
      auto it = depth_of.find(idx);
      if (it == depth_of.end()) {
        std::cout << (G->U[idx] ? '.' : '#');
      } else {
        std::cout << COLORS[agent_id % NUM_COLORS] << (it->second % 10) << RESET;
      }
    }
    std::cout << '\n';
  }
}


MDD MDD::get_mdd_with_wait() {
  MDD new_mdd;
  new_mdd.frontiers.resize(frontiers.size());
  new_mdd.frontiers[0] = frontiers[0];

  for (int t = 1; t < frontiers.size(); t++) {
    new_mdd.frontiers[t] = frontiers[t - 1];
  }
  return new_mdd;
}


MDD MDD::get_mdd_with_wait_at_time_1() {
  MDD new_mdd;
  new_mdd.frontiers.resize(frontiers.size());
  new_mdd.frontiers[0] = frontiers[0];
  new_mdd.frontiers[1] = frontiers[1];

  for (int t = 2; t < frontiers.size(); t++) {
    new_mdd.frontiers[t] = frontiers[t - 1];
  }
  return new_mdd;
}

bool MDD::check_joint_mdd_conflict(MDD& other_mdd, Graph* G, int horizon, int dt_me, int dt_other) {
    if (horizon + dt_me > frontiers.size()) {
      throw std::runtime_error("this->frontiers is too small for joint comparison. frontiers.size: " +
                               std::to_string(frontiers.size()));
    }
    if (horizon + dt_other > other_mdd.frontiers.size()) {
      throw std::runtime_error("other->frontiers is too small for joint comparison. frontiers.size: " +
                               std::to_string(other_mdd.frontiers.size()));
    }

    std::vector<Vertex*> u1_self_vector({ nullptr });
    std::vector<Vertex*> u2_self_vector({ nullptr });

    thread_local std::vector<std::pair<Vertex*, Vertex*>> current_layer;
    thread_local std::vector<std::pair<Vertex*, Vertex*>> next_layer;
    
    current_layer.clear();
    for (Vertex* v1 : frontiers[dt_me]) {
      for (Vertex* v2 : other_mdd.frontiers[dt_other]) {
        if (v1 != v2)
          current_layer.push_back({ v1, v2 });
      }
    }
    if (current_layer.size() == 0) return true;

    // Allocate once per thread, reuse endlessly
    thread_local std::vector<bool> in_f1;
    thread_local std::vector<bool> in_f2;
    // Assuming G->V.size() is known/accessible, otherwise resize on first use
    if (in_f1.empty()) {
        in_f1.resize(G->V.size(), false);
        in_f2.resize(G->V.size(), false);
    }

    bool is_1_waiting = false;
    bool is_2_waiting = false;

    for (int time = 1; time < horizon; ++time) {
        int t1 = time + dt_me;
        int t2 = time + dt_other;
        next_layer.clear();

        is_1_waiting = frontiers[t1].size() == 1 && frontiers[t1-1].size() == 1 && frontiers[t1][0] == frontiers[t1-1][0];
        is_2_waiting = other_mdd.frontiers[t2].size() == 1 && other_mdd.frontiers[t2-1].size() == 1 && other_mdd.frontiers[t2][0] == other_mdd.frontiers[t2-1][0];

        for (Vertex* v : frontiers[t1]) in_f1[v->id] = true;
        for (Vertex* v : other_mdd.frontiers[t2]) in_f2[v->id] = true;

        for (const auto& pair : current_layer) {
            Vertex* u1 = pair.first;
            Vertex* u2 = pair.second;

            std::vector<Vertex*>* u1_candidates = &u1->neighbor;
            std::vector<Vertex*>* u2_candidates = &u2->neighbor;

            if (is_1_waiting) { u1_self_vector[0] = u1; u1_candidates = &u1_self_vector; }
            if (is_2_waiting) { u2_self_vector[0] = u2; u2_candidates = &u2_self_vector; }

            for (Vertex* v1 : *u1_candidates) {
                if (!in_f1[v1->id]) continue;

                for (Vertex* v2 : *u2_candidates) {
                    if (!in_f2[v2->id]) continue;

                    if (v1 == v2) continue; // Vertex conflict
                    if (u1 == v2 && u2 == v1) continue; // Swap conflict

                    next_layer.push_back({v1, v2});
                }
            }
        }

        // Cleanup
        for (Vertex* v : frontiers[t1]) in_f1[v->id] = false;
        for (Vertex* v : other_mdd.frontiers[t2]) in_f2[v->id] = false;

        if (next_layer.empty()) return true; 

        // std::pair has a built-in operator< that compares .first then .second.
        // It is heavily optimized by the compiler.
        std::sort(next_layer.begin(), next_layer.end());
        next_layer.erase(std::unique(next_layer.begin(), next_layer.end()), next_layer.end());

        current_layer.swap(next_layer);
    }

    return false; 
}
