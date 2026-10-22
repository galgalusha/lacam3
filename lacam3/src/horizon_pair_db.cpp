#include "../include/horizon_pair_db.hpp"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_map>


// Prints median/average/p90/p95 of the number of entries per conflict map.
static void print_conflict_entry_stats(std::vector<uint32_t> entries_per_map,
                                       const std::vector<uint64_t>& count_per_penalty) {
  std::cout << "Conflicts per penalty:";
  for (size_t p = 0; p < count_per_penalty.size(); p++) {
    std::cout << " [" << p << "]=" << count_per_penalty[p];
  }
  std::cout << std::endl;

  if (entries_per_map.empty()) return;
  std::sort(entries_per_map.begin(), entries_per_map.end());

  const size_t n = entries_per_map.size();
  auto percentile = [&](double p) {
    size_t idx = std::min(n - 1, static_cast<size_t>(p * n));
    return entries_per_map[idx];
  };

  double sum = 0;
  for (uint32_t v : entries_per_map) sum += v;
  double average = sum / (double)n;
  double median = (n % 2 == 0)
      ? (entries_per_map[n / 2 - 1] + entries_per_map[n / 2]) / 2.0
      : entries_per_map[n / 2];

  std::cout << "Conflict map entry stats (n=" << n << "): "
            << "average=" << std::fixed << std::setprecision(2) << average
            << ", median=" << median
            << ", p90=" << percentile(0.90)
            << ", p95=" << percentile(0.95) << std::endl;
}

HorizonPairDB::HorizonPairDB(Graph* _G, std::string _name) : G(_G), V_SIZE(_G->V.size()), name(_name) {}


// ---------------------------------------------------------------------------
// Deserialization
// ---------------------------------------------------------------------------

bool HorizonPairDB::read_header(std::ifstream& in, HorizonPairDBFileHeader& header) {
  in.read(reinterpret_cast<char*>(&header.num_mdds), sizeof(header.num_mdds));
  in.read(reinterpret_cast<char*>(&header.horizon), sizeof(header.horizon));
  in.read(reinterpret_cast<char*>(&header.margin), sizeof(header.margin));
  in.read(reinterpret_cast<char*>(&header.offset_mdd_by_id), sizeof(header.offset_mdd_by_id));
  in.read(reinterpret_cast<char*>(&header.offset_mdd_id_by_v_g), sizeof(header.offset_mdd_id_by_v_g));
  in.read(reinterpret_cast<char*>(&header.offset_conflicts), sizeof(header.offset_conflicts));
  return static_cast<bool>(in);
}


bool HorizonPairDB::read_mdd_by_id_section(std::ifstream& in) {
  std::cout << "Loading MDDs..." << std::endl;
  mdd_by_id.resize(mdd_count);
  for (uint32_t id = 0; id < mdd_count; id++) {
    if (id % 256 == 0 || id + 1 == mdd_count) print_horizon_pair_db_progress_bar(id + 1, mdd_count);
  }
  std::cout << std::endl;

  uint32_t summary;
  in.read(reinterpret_cast<char*>(&summary), sizeof(summary));
  if (!in || summary != mdd_count) {
    std::cerr << "HorizonPairDB::read_mdd_by_id_section: summary mismatch, expected "
              << mdd_count << " got " << summary << std::endl;
    return false;
  }
  return true;
}

bool HorizonPairDB::read_mdd_id_by_v_g_section(std::ifstream& in) {
  std::cout << "Loading mdd_id_by_v_g..." << std::endl;
  mdd_id_by_v_g.assign(V_SIZE * V_SIZE, 0);
  in.read(reinterpret_cast<char*>(mdd_id_by_v_g.data()),
          mdd_id_by_v_g.size() * sizeof(uint32_t));
  print_horizon_pair_db_progress_bar(1, 1);
  std::cout << std::endl;
  return static_cast<bool>(in);
}

bool HorizonPairDB::read_conflicts_section(std::ifstream& in) {
  std::cout << "Loading conflicts..." << std::endl;
  uint32_t num_conflicts;
  in.read(reinterpret_cast<char*>(&num_conflicts), sizeof(num_conflicts));
  if (!in || num_conflicts != mdd_count) {
    std::cerr << "HorizonPairDB::read_conflicts_section: count mismatch, expected "
              << mdd_count << " got " << num_conflicts << std::endl;
    return false;
  }

  penalties.assign(num_conflicts, {});
  std::vector<uint32_t> entries_per_map;
  entries_per_map.reserve(num_conflicts);
  // Max possible penalty is MARGIN + 1
  std::vector<uint64_t> count_per_penalty(MARGIN + 2, 0);
  for (uint32_t id = 0; id < num_conflicts; id++) {
    uint32_t num_entries;
    in.read(reinterpret_cast<char*>(&num_entries), sizeof(num_entries));
    if (!in) return false;
    for (uint32_t e = 0; e < num_entries; e++) {
      uint32_t key;
      uint8_t value;
      in.read(reinterpret_cast<char*>(&key), sizeof(key));
      in.read(reinterpret_cast<char*>(&value), sizeof(value));
      if (!in) return false;
      if (value >= count_per_penalty.size()) count_per_penalty.resize(value + 1, 0);
      count_per_penalty[value]++;
      penalties[id].push_back({key, value});
      penalties[key].push_back({id, value});
    }
    entries_per_map.push_back(num_entries);
    if (id % 256 == 0 || id + 1 == num_conflicts) print_horizon_pair_db_progress_bar(id + 1, num_conflicts);
  }
  std::cout << std::endl;
  print_conflict_entry_stats(entries_per_map, count_per_penalty);

  return true;
}

bool HorizonPairDB::load_from_file() {
  const std::string path = HORIZON_PAIR_DB_ROOT_FOLDER + name + HORIZON_PAIR_DB_FILE_EXTENSION;
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cerr << "HorizonPairDB::load_from_file: failed to open " << path << " for reading" << std::endl;
    return false;
  }

  HorizonPairDBFileHeader header;
  if (!read_header(in, header)) {
    std::cerr << "HorizonPairDB::load_from_file: failed to read header from " << path << std::endl;
    return false;
  }
  mdd_count = header.num_mdds;
  HORIZON = header.horizon;
  MARGIN = header.margin;

  in.seekg(header.offset_mdd_by_id);
  if (!read_mdd_by_id_section(in)) return false;

  in.seekg(header.offset_mdd_id_by_v_g);
  if (!read_mdd_id_by_v_g_section(in)) return false;

  in.seekg(header.offset_conflicts);
  if (!read_conflicts_section(in)) return false;

  std::cout << "Loaded HorizonPairDB from " << path << std::endl;
  return true;
}

