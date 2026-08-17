#pragma once

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

// 40-bit key: 10 bits each for i_goal, j_goal, i_start, j_start (MSB to LSB)
using JointKey = uint64_t;

inline JointKey to_joint_key(uint16_t i_goal, uint16_t j_goal, uint16_t i_start, uint16_t j_start) {
  return ((JointKey)(i_goal  & 0x3FF) << 30) |
         ((JointKey)(j_goal  & 0x3FF) << 20) |
         ((JointKey)(i_start & 0x3FF) << 10) |
         ((JointKey)(j_start & 0x3FF));
}

// Appends entries as raw binary to path (supports incremental flushing)
void append_entries(const std::string& path, const std::vector<PairEntry>& entries);

// Reads all PairEntry records from a binary file and writes them as CSV
void to_csv(const std::string& bin_path, const std::string& csv_path);

// For each file in goal_dir, merges it with the matching file in base_dir (goal overrides base on conflict)
// and writes the sorted result back into base_dir, replacing the original.
void merge_goal_folder(const std::string& base_dir, const std::string& goal_dir);
