#include "../include/arm.hpp"
#include "../include/scatter.hpp"

#include <algorithm>
#include <iostream>

std::vector<float> PATH_RATIO = { 0.6, 0.55, 0.5, 0.45, 0.4 };

// ---- ArmConfig ----

std::string ArmConfig::str() const
{
  return "[" + std::string(scatter_type == ST_Scatter ? "Scatter" : "WaitScatter")
       + ", " + std::to_string(margin) + "]";
}

// ---- Arm ----

Arm::Arm(ArmConfig _config)
    : config(_config), improvement_count(0), pulls(0), idx_path_ratio(0)
{}

float Arm::get_path_ratio()
{
  float ratio = PATH_RATIO[idx_path_ratio];
  idx_path_ratio = (idx_path_ratio + 1) % PATH_RATIO.size();
  return ratio;
}

float Arm::get_score(int total_pulls) const
{
  if (pulls == 0) return std::numeric_limits<float>::infinity();
  float avg = static_cast<float>(improvement_count) / static_cast<float>(pulls);
  float exploration = std::sqrt(2.0f * std::log(static_cast<float>(total_pulls)) / static_cast<float>(pulls));
  return avg + exploration;
}

// ---- ScatterMAB ----

ScatterMAB::ScatterMAB(std::vector<ArmConfig> configs) : total_pulls(0)
{
  for (auto &cfg : configs) arms.emplace_back(cfg);
  cache.resize(arms.size());
}

ScatterMAB::~ScatterMAB()
{
  for (auto &per_arm : cache)
    for (auto &[key, ptr] : per_arm) delete ptr;
}

int ScatterMAB::choose_arm()
{
  std::lock_guard<std::mutex> lock(mtx);
  int tp = std::max(1, total_pulls);
  int best = 0;
  float best_score = -1.0f;
  std::cout << "Arm Scores:";
  for (int i = 0; i < static_cast<int>(arms.size()); ++i) {
    float s = arms[i].get_score(tp);
    std::cout << " [" << i << "|" << arms[i].config.str() << "] " << s;
    if (s > best_score) { best_score = s; best = i; }
  }
  std::cout << std::endl;
  return best;
}

void ScatterMAB::record_pull(int arm_idx)
{
  std::lock_guard<std::mutex> lock(mtx);
  ++arms[arm_idx].pulls;
  ++total_pulls;
}

void ScatterMAB::record_improvement(int arm_idx)
{
  std::lock_guard<std::mutex> lock(mtx);
  ++arms[arm_idx].improvement_count;
  std::cout << "ARM " << arms[arm_idx].config.str() << " - improvement recorded" << std::endl;
}

IScatter *ScatterMAB::get_cached(int arm_idx, const Config &starts)
{
  if (arm_idx < 0 || arm_idx >= static_cast<int>(cache.size())) return nullptr;
  std::lock_guard<std::mutex> lock(mtx);
  auto it = cache[arm_idx].find(starts);
  return it != cache[arm_idx].end() ? it->second : nullptr;
}

void ScatterMAB::set_cached(int arm_idx, const Config &starts, IScatter *scatter)
{
  if (arm_idx < 0 || arm_idx >= static_cast<int>(cache.size())) return;
  std::lock_guard<std::mutex> lock(mtx);
  cache[arm_idx][starts] = scatter;
}

// ---- Global MAB instances ----

ScatterMAB main_scatter_mab({
  {ST_Scatter    , 10},
  {ST_Scatter    , 90},
  {ST_WaitScatter, 20},
  {ST_WaitScatter, 45},
});

ScatterMAB refiners_scatter_mab({
  {ST_WaitScatter, 10},
  {ST_WaitScatter, 20},
  {ST_Scatter    , 40},
  {ST_Scatter    , 90},
});
