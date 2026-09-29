#include "../include/mdd_registry.hpp"



MDDRegistry::MDDRegistry(int N, uint32_t mdd_count) : NO_AGENT(N), NO_MDD(mdd_count + 1) {
  agent_to_mdd_id_now.assign(N, NO_MDD);
  agent_to_mdd_id_next.assign(N, NO_MDD);
  mdd_id_to_agent_now.assign(mdd_count, NO_AGENT);
  mdd_id_to_agent_next.assign(mdd_count, NO_AGENT);
}


void MDDRegistry::register_agent_now(int agent, uint32_t mdd_id) {
  agent_to_mdd_id_now[agent] = mdd_id;
  mdd_id_to_agent_now[mdd_id] = agent;
}


void MDDRegistry::register_agent_next(int agent, uint32_t new_mdd_id) {
  // Clean old
  uint32_t old_mdd = agent_to_mdd_id_next[agent];
  if (old_mdd != NO_MDD && mdd_id_to_agent_next[old_mdd] == agent) {
    mdd_id_to_agent_next[old_mdd] = NO_AGENT;
  }
  // Set new
  agent_to_mdd_id_next[agent] = new_mdd_id;
  mdd_id_to_agent_next[new_mdd_id] = agent;
}


void MDDRegistry::clear() {
  for (int agent = 0; agent < agent_to_mdd_id_now.size(); ++agent) {
    uint32_t mdd_now = agent_to_mdd_id_now[agent];
    if (mdd_now != NO_MDD) {
      mdd_id_to_agent_now[mdd_now] = NO_AGENT;
      agent_to_mdd_id_now[agent] = NO_MDD;
    }

    // Clear _next registrations
    uint32_t mdd_next = agent_to_mdd_id_next[agent];
    if (mdd_next != NO_MDD) {
      mdd_id_to_agent_next[mdd_next] = NO_AGENT;
      agent_to_mdd_id_next[agent] = NO_MDD;
    }
  }
}

