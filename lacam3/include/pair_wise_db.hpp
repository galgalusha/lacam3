#pragma once

#include <cereal/archives/binary.hpp>
#include <cereal/types/vector.hpp>
#include <cstdint>
#include <sstream>
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

  template <class Archive>
  void serialize(Archive& archive) {
    archive(i_start, j_start, dh);
  }
};

// Appends entries as raw binary to path (supports incremental flushing)
void append_entries(const std::string& path, const std::vector<PairEntry>& entries);

// Serializes entries to a binary string for later storage in RocksDB
std::string serialize_for_rocksdb(const std::vector<PairEntry>& entries);
