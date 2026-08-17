#include "../include/pair_wise_h.hpp"


static DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}


PairWiseHeuristic::PairWiseHeuristic(Graph* _G, std::string _name)
    : G(_G), name(_name), D(create_dist_table(_G)) {}


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

