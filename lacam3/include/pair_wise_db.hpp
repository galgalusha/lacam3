#pragma once

#include "graph.hpp"

#include <absl/container/flat_hash_map.h>
#include <cstdint>
#include <string>
#include <vector>

using PairKey = uint32_t;

inline PairKey to_key(uint16_t vertex_i, uint16_t vertex_j) {
  return ((uint32_t)vertex_i << 16) | vertex_j;
}

struct PairEntry {
  uint16_t i_start;
  uint16_t j_start;
  uint16_t dh;
};

// Lightweight handle into a loaded table; cheap to copy by value.
struct PairDHTable {
  const absl::flat_hash_map<PairKey, uint16_t>* data = nullptr;
  bool flipped = false;

  // Returns dh for (a->g1, b->g2). Swaps lookup key when table was stored under reversed goal order.
  uint16_t get(Vertex* a, Vertex* b) const {
    if (!data) return 0;
    PairKey key = flipped ? to_key((uint16_t)b->id, (uint16_t)a->id)
                          : to_key((uint16_t)a->id, (uint16_t)b->id);
    auto it = data->find(key);
    return it != data->end() ? it->second : 0;
  }
};

// Appends entries as raw binary to path (supports incremental flushing)
void append_entries(const std::string& path, const std::vector<PairEntry>& entries);

// Reads all PairEntry records from a binary file and writes them as CSV
void to_csv(const std::string& bin_path, const std::string& csv_path);

// For each file in goal_dir, merges it with the matching file in base_dir (goal overrides base on conflict)
// and writes the sorted result back into base_dir, replacing the original.
void merge_goal_folder(const std::string& base_dir, const std::string& goal_dir);
