#include "../include/pair_wise_db.hpp"

#include <algorithm>


static DistTable* create_dist_table(Graph* G) {
  Config goals = G->V;
  Instance* ins = new Instance(G, goals, goals, goals.size());
  auto D = new DistTable(ins);
  return D;
}


PairWiseDB::PairWiseDB(Graph* _G, std::string _name)
    : G(_G), name(_name), D(create_dist_table(_G)) {}


uint8_t PairWiseDB::get(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) const {
  bool flipped = i_goal > j_goal;
  uint16_t lo  = flipped ? j_goal  : i_goal;
  uint16_t hi  = flipped ? i_goal  : j_goal;
  uint16_t is  = flipped ? j_start : i_start;
  uint16_t js  = flipped ? i_start : j_start;

  auto it = pair_data.find(to_goals_key(lo, hi));
  if (it == pair_data.end()) return 0;
  if (is >= MAX_VERTICES || js >= MAX_VERTICES) return 0;
  const auto& ranges = it->second[is];
  if (ranges.empty()) return 0;

  // Binary search for the last range whose j_start <= js
  auto cmp = [](uint16_t val, const RangeEntry& r) { return val < r.j_start; };
  auto rit = std::upper_bound(ranges.begin(), ranges.end(), js, cmp);
  if (rit == ranges.begin()) return 0;
  --rit;
  return js < (uint16_t)(rit->j_start + rit->range) ? rit->dh : 0;
}

