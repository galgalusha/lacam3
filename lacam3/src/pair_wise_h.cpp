#include "../include/pair_wise_h.hpp"

#include <iostream>

PairWiseHeuristic::PairWiseHeuristic(const Instance &_ins)
    : ins(&_ins), G(_ins.G), D_goals(_ins, TO_GOALS), D_starts(_ins, TO_STARTS)
{
}

bool PairWiseHeuristic::can_interfere(int i, int j) { 
  // 1. Retrieve the pairwise start and goal distances
  // \Delta^s_{ij} : distance from start i to start j
  int dist_s_ij = D_starts.get(i, ins->starts[j]->id); 
  
  // \Delta^g_{ij} : distance from goal i to goal j
  int dist_g_ij = D_goals.get(i, ins->goals[j]->id);   

  // 2. Retrieve the shortest path lengths for both agents
  // \Delta^{sg}_{ii} : distance from start i to goal i
  int dist_sg_ii = D_starts.get(i, ins->goals[i]->id); 
  
  // \Delta^{sg}_{jj} : distance from start j to goal j
  int dist_sg_jj = D_starts.get(j, ins->goals[j]->id); 

  // 3. Evaluate the Psi constraint
  int psi_ij = dist_s_ij + dist_g_ij - dist_sg_ii - dist_sg_jj;
  if (psi_ij > 0) {
    return false; // The agents cannot possibly interfere
  }

  // 4. Retrieve cross-distances for the Lambda constraint
  // \Delta^{sg}_{ji} : distance from start j to goal i
  int dist_sg_ji = D_starts.get(j, ins->goals[i]->id); 
  
  // \Delta^{sg}_{ij} : distance from start i to goal j
  int dist_sg_ij = D_starts.get(i, ins->goals[j]->id); 

  // 5. Evaluate the Lambda constraint
  int lambda_ij = dist_sg_ii - dist_sg_ji;
  int lambda_ji = dist_sg_jj - dist_sg_ij;

  if (lambda_ij + lambda_ji < 0) {
    return false; // The agents are unconstrained and cannot interfere
  }

  // 6. If both geometric constraints fail to prove safety, they may interfere
  return true; 
}

void PairWiseHeuristic::construct() { 

  int count = 0;

  for (int i = 0; i < ins->N; i++) {
    for (int j = i + 1; j < ins->N; j++) {

      if (can_interfere(i, j)) count++;
    }
  }


  std::cout << "Num of interfering pairs: " << count << std::endl; 
}

void PairWiseHeuristic::test() {

  std::vector<std::string> grid = {
    "....",
    "....",
    "....",
  };
  Graph* G = new Graph(grid);

  auto coord = [G](int row, int col) {
    auto index = G->width * row + col;
    return G->U[index];
  };

  // Test 1
  {
    Config starts = { coord(0, 0) , coord(2, 0) };
    Config goals  = { coord(0, 3) , coord(2, 3) };

    Instance* ins = new Instance(G, starts, goals, starts.size());
    auto pwh = PairWiseHeuristic(*ins);
    bool can_interfere = pwh.can_interfere(0, 1);
    std::cout << "[Test 1] can interfere: " << (can_interfere ? "yes" : "no") << std::endl;
  }

  // Test 2
  {
    Config starts = { coord(0, 0) , coord(2, 2) };
    Config goals  = { coord(1, 1) , coord(1, 3) };

    Instance* ins = new Instance(G, starts, goals, starts.size());
    auto pwh = PairWiseHeuristic(*ins);
    bool can_interfere = pwh.can_interfere(0, 1);
    std::cout << "[Test 2] can interfere: " << (can_interfere ? "yes" : "no") << std::endl;
  }
}
