#pragma once

#include "graph.hpp"
#include "dist_table.hpp"
#include "instance.hpp"
#include "thread_pool.hpp"

#include <absl/container/flat_hash_map.h>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>
#include <mutex>
#include <vector>


struct PairWiseDB;

/**
 * For a specic agent i located at vertex v_from and a specific adjacent vertex v_to
 * where the agent can move from v_from to v_to in a single move (there are exactly 4 such
 * vertices for left, right, up, down), MoveQuality = D(i, v_from)-D(i, v_to) 
 * where D is a distance table (DistTable) that for an agent i and a vertex gives its true
 * distance to goal.
 * There should be at least one optimal move with MoveQuality=-1 which gets the agent
 * one step closer to its goal. If v_to is not a vertex (the map is blocked for that move),
 * then MoveQuality=BLOCKED_MOVE.
 */
using MoveQuality = int8_t;
const MoveQuality BLOCKED_MOVE = 127;


struct AgentProfile {
  MoveQuality left;
  MoveQuality right;
  MoveQuality up;
  MoveQuality down;
};


/**
 * This is the key to our database which captures a pattern of two agents:
 * the main agent and an other agent that is shifted from the main agent
 * by dx (horizontally) and dy (vertically). If the database is scoped to
 * a radius of 2, then -2<=dx<=2, -2<=dy<=2.
 */
struct Pattern {
  int8_t dx;
  int8_t dy;
  AgentProfile agent;
  AgentProfile other_agent;

  struct PatternHash {
    std::size_t operator()(const Pattern& p) const {
        std::string_view view(reinterpret_cast<const char*>(&p), sizeof(Pattern));
        return std::hash<std::string_view>{}(view);
    }
  };

  // required by absl::flat_hash_map (Pattern may contain padding, so compare byte-wise
  // consistently with PatternHash rather than relying on the compiler-generated operator==).
  bool operator==(const Pattern& other) const {
    std::string_view lhs(reinterpret_cast<const char*>(this), sizeof(Pattern));
    std::string_view rhs(reinterpret_cast<const char*>(&other), sizeof(Pattern));
    return lhs == rhs;
  }

  std::string str() const;
};


const uint8_t NO_DATA = 255;

/**
 * The database maps a pattern to a penalty, which is how much extra
 * steps will the two agents do if they are planned jointly minus the
 * the total amount of steps they need to reach their goals individually
 * ignoring possible conflicts. If two agents cannot interfere each other
 * reaching their goals, then the entry is 0.
 * Agents represented by their IDs which are integers.
 */
struct PatternDB {
  const static int RADIUS;
  Instance* ins = nullptr;
  DistTable* D;

  absl::flat_hash_map<Pattern, uint8_t, Pattern::PatternHash> data;

  // returns data[pattern], or NO_DATA if pattern does not exist.
  uint8_t get(Pattern& pattern);

  // inserts (pattern, value) into data. If pattern already exists with a
  // different value, prints the conflict to std::cout and keeps the
  // existing value.
  uint8_t set(Pattern& pattern, uint8_t value);

  // stores ins and (re)initializes D with the true, agent-agnostic all-pairs
  // distance table (D->get(u_id, v) is the distance between vertex u_id and v).
  void set_instance(Instance* ins);

  // builds a Pattern for (agent, other_agent) located at (v, other_v) using D.
  Pattern pattern(int agent, int other_agent, Vertex* v, Vertex* other_v);

  // writes data to a binary file. Returns false on failure.
  bool save_to_file(const std::string& filename) const;

  // clears data and loads it from a binary file previously written by
  // save_to_file. Returns false on failure (data is left empty in that case).
  bool load_from_file(const std::string& filename);

  void populate_from(PairWiseDB* pair_db);

private:
  // builds an all-pairs-to-vertex-id distance table: D->get(u_id, v) is the
  // true distance between the vertex with id u_id and v.
  static DistTable* create_dist_table(Graph* G);
};

