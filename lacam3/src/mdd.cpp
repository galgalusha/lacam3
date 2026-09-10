#include "../include/mdd.hpp"
#include "../include/drawing.hpp"

#include <unordered_map>


void MDD::populate(DistTable* D, Vertex* v_i, Vertex* v_g, int horizon) {
  frontiers.resize(horizon + 1);

  frontiers[0].push_back(v_i);
  int D_t_minus_1 = D->get(v_i->id, v_g->id);

  int t;
  for (t = 1; t <= horizon; t++) { 
    bool was_goal_inserted_for_this_depth = false;
    for (Vertex* v_t_minus_1 : frontiers[t - 1]) {
      
      // // 2. Goal Sink Fix: If the agent reached the goal early, it must wait in place.
      // if (v_t_minus_1 == v_g) {
      //   // Ensure we don't add the goal multiple times if multiple paths converge
      //   if (!was_goal_inserted_for_this_depth) {
      //     mdd[t].push_back(v_g);
      //     was_goal_inserted_for_this_depth = true;
      //   }
      //   continue; // Skip neighbor expansion
      // }

      for (Vertex* v_t : v_t_minus_1->neighbor) {
        // Verify optimal progression
        if (D->get(v_t->id, v_g->id) == D_t_minus_1 - 1) {
          
          // 3. Duplicate Prevention: Since K is tiny, layer sizes are small. 
          // A linear std::find is faster than allocating a std::unordered_set.
          if (std::find(frontiers[t].begin(), frontiers[t].end(), v_t) == frontiers[t].end()) {
            frontiers[t].push_back(v_t);
          }
        }
      }
    }
    D_t_minus_1--;
    if (D_t_minus_1 < 0) { break; }
  }
  frontiers.resize(t);
}


void MDD::render(Instance* ins) {
  int i = agent_id;
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
        std::cout << COLORS[i % NUM_COLORS] << (it->second % 10) << RESET;
      }
    }
    std::cout << '\n';
  }
}