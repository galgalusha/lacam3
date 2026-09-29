#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>


struct MDDRegistry
{
  MDDRegistry(int N, uint32_t mdd_count);

  const uint32_t NO_MDD;
  const int NO_AGENT;

  std::vector<uint32_t> agent_to_mdd_id_now;
  std::vector<uint32_t> agent_to_mdd_id_next;

  std::vector<int> mdd_id_to_agent_now;
  std::vector<int> mdd_id_to_agent_next;

  void register_agent_now(int agent, uint32_t mdd_id);
  void register_agent_next(int agent, uint32_t new_mdd_id);

  void clear();
};

