#pragma once

#include <cstdint>
#include <string>
#include <vector>

using PairKey = uint32_t;

inline PairKey to_bin_key(uint16_t vertex_i, uint16_t vertex_j) {
  return ((uint32_t)vertex_i << 16) | vertex_j;
}

struct BinEntry {
  uint16_t i_start;
  uint16_t j_start;
  uint16_t dh;
};

// Appends entries as raw binary to path (supports incremental flushing)
void append_entries(const std::string& path, const std::vector<BinEntry>& entries);

// Reads all PairEntry records from a binary file and writes them as CSV
void to_csv(const std::string& bin_path, const std::string& csv_path);

// For each file in goal_dir, merges it with the matching file in base_dir (goal overrides base on conflict)
// and writes the sorted result back into base_dir, replacing the original.
void merge_goal_folder(const std::string& base_dir, const std::string& goal_dir);
