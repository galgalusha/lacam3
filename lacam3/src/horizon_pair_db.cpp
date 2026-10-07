#include "../include/horizon_pair_db.hpp"
#include "../include/moves.hpp"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <unordered_map>


// Prints median/average/p90/p95 of the number of entries per conflict map.
static void print_conflict_entry_stats(std::vector<uint32_t> entries_per_map) {
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
  in.read(reinterpret_cast<char*>(&header.offset_mdd_by_id), sizeof(header.offset_mdd_by_id));
  in.read(reinterpret_cast<char*>(&header.offset_mdd_id_by_v_g), sizeof(header.offset_mdd_id_by_v_g));
  in.read(reinterpret_cast<char*>(&header.offset_conflicts), sizeof(header.offset_conflicts));
  in.read(reinterpret_cast<char*>(&header.offset_constrained_move_conflicts), sizeof(header.offset_constrained_move_conflicts));
  return static_cast<bool>(in);
}

// Frontier vertices are stored as v->id (index into G->V), not v->index.
bool HorizonPairDB::read_mdd(std::ifstream& in, MDD& mdd) {
  uint8_t num_frontiers;
  in.read(reinterpret_cast<char*>(&num_frontiers), sizeof(num_frontiers));
  mdd.frontiers.assign(num_frontiers, {});
  for (uint8_t t = 0; t < num_frontiers; t++) {
    uint8_t num_vertices;
    in.read(reinterpret_cast<char*>(&num_vertices), sizeof(num_vertices));
    mdd.frontiers[t].resize(num_vertices);
    for (uint8_t i = 0; i < num_vertices; i++) {
      uint16_t vid;
      in.read(reinterpret_cast<char*>(&vid), sizeof(vid));
      mdd.frontiers[t][i] = G->V[vid];  // v->id doubles as the index into G->V
    }
  }
  uint8_t sentinel;
  in.read(reinterpret_cast<char*>(&sentinel), sizeof(sentinel));
  if (!in) return false;
  if (sentinel != HORIZON_PAIR_DB_MDD_SENTINEL) {
    std::cerr << "HorizonPairDB::read_mdd: sentinel mismatch, file is corrupt" << std::endl;
    return false;
  }
  return true;
}

bool HorizonPairDB::read_mdd_by_id_section(std::ifstream& in) {
  std::cout << "Loading MDDs..." << std::endl;
  mdd_by_id.resize(mdd_count);
  for (uint32_t id = 0; id < mdd_count; id++) {
    if (!read_mdd(in, mdd_by_id[id])) return false;
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
      penalties[id].push_back({key, value});
      penalties[key].push_back({id, value});
    }
    entries_per_map.push_back(num_entries);
    if (id % 256 == 0 || id + 1 == num_conflicts) print_horizon_pair_db_progress_bar(id + 1, num_conflicts);
  }
  std::cout << std::endl;
  print_conflict_entry_stats(entries_per_map);

  return true;
}

bool HorizonPairDB::read_move_constrained_conflicts_section(std::ifstream& in) {
  std::cout << "Loading constrained move conflicts..." << std::endl;
  uint32_t num_entries_outer;
  in.read(reinterpret_cast<char*>(&num_entries_outer), sizeof(num_entries_outer));
  if (!in || num_entries_outer != mdd_count * NUM_OF_MOVES) {
    std::cerr << "HorizonPairDB::read_move_constrained_conflicts_section: count mismatch, expected "
              << mdd_count * NUM_OF_MOVES << " got " << num_entries_outer << std::endl;
    return false;
  }

  constrained_move_penalties.assign(num_entries_outer, {});
  for (uint32_t id = 0; id < num_entries_outer; id++) {
    uint32_t num_entries;
    in.read(reinterpret_cast<char*>(&num_entries), sizeof(num_entries));
    if (!in) return false;
    auto& entries = constrained_move_penalties[id];
    entries.reserve(num_entries);
    for (uint32_t e = 0; e < num_entries; e++) {
      uint32_t key;
      uint8_t value;
      in.read(reinterpret_cast<char*>(&key), sizeof(key));
      in.read(reinterpret_cast<char*>(&value), sizeof(value));
      if (!in) return false;
      entries.push_back({key, value});
    }
    if (id % 256 == 0 || id + 1 == num_entries_outer) print_horizon_pair_db_progress_bar(id + 1, num_entries_outer);
  }
  std::cout << std::endl;
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

  in.seekg(header.offset_mdd_by_id);
  if (!read_mdd_by_id_section(in)) return false;

  in.seekg(header.offset_mdd_id_by_v_g);
  if (!read_mdd_id_by_v_g_section(in)) return false;

  in.seekg(header.offset_conflicts);
  if (!read_conflicts_section(in)) return false;

  in.seekg(header.offset_constrained_move_conflicts);
  if (!read_move_constrained_conflicts_section(in)) return false;

  std::cout << "Loaded HorizonPairDB from " << path << std::endl;
  return true;
}

// ---------------------------------------------------------------------------
// Comparison
// ---------------------------------------------------------------------------

static bool mdd_penalty_less(const HorizonPairDB::MDD_Penalty& a, const HorizonPairDB::MDD_Penalty& b) {
  return a.mdd_id < b.mdd_id;
}

void HorizonPairDB::compare_to_other_db(HorizonPairDB& other_db) {
  if (mdd_count != other_db.mdd_count) {
    std::cerr << "compare_to_other_db: mdd_count mismatch, this=" << mdd_count
              << " other=" << other_db.mdd_count << std::endl;
    return;
  }
  if (V_SIZE != other_db.V_SIZE) {
    std::cerr << "compare_to_other_db: V_SIZE mismatch, this=" << V_SIZE
               << " other=" << other_db.V_SIZE << std::endl;
    return;
  }

  // Walk every (v, g) pair just like generate_mdds() does, and the first time
  // an mdd_id is seen, record which mdd_id it corresponds to in other_db.
  // mdd_ids are not comparable directly across DBs since their values depend
  // on thread scheduling during generation, but the (v, g) they're keyed by is.
  std::vector<uint32_t> other_id_of(mdd_count, 0);
  std::vector<uint32_t> this_id_of(mdd_count, 0);
  uint32_t num_visited = 0;

  std::cout << "Comparing DBs..." << std::endl;
  const size_t total = V_SIZE * V_SIZE;
  size_t done = 0;
  for (size_t v = 0; v < V_SIZE; v++) {
    for (size_t g = 0; g < V_SIZE; g++) {
      size_t idx = v * V_SIZE + g;
      uint32_t this_mdd_id = mdd_id_by_v_g[idx];
      uint32_t other_mdd_id = other_db.mdd_id_by_v_g[idx];
      other_id_of[this_mdd_id] = other_mdd_id;
      this_id_of[other_mdd_id] = this_mdd_id;
      num_visited++;
      done++;
      if (done % 4096 == 0 || done == total) print_horizon_pair_db_progress_bar(done, total);
    }
  }
  std::cout << std::endl;

  if (num_visited != mdd_count) {
    std::cerr << "compare_to_other_db: only " << num_visited << "/" << mdd_count
               << " mdd_ids were reachable from mdd_id_by_v_g" << std::endl;
    return;
  }

  for (uint32_t mdd_id = 0; mdd_id < mdd_count; mdd_id++) {
    uint32_t other_mdd_id = other_id_of[mdd_id];

    // Translate this DB's other_mdd_id references into other_db's id space
    // before comparing, since nested ids are just as DB-specific as the top one.
    std::vector<MDD_Penalty> this_entries;
    this_entries.reserve(penalties[mdd_id].size());
    for (const MDD_Penalty& entry : penalties[mdd_id])
      this_entries.push_back({other_id_of[entry.mdd_id], entry.penalty});

    std::vector<MDD_Penalty> other_entries = other_db.penalties[other_mdd_id];
    std::sort(this_entries.begin(), this_entries.end(), mdd_penalty_less);
    std::sort(other_entries.begin(), other_entries.end(), mdd_penalty_less);

    if (this_entries.size() == other_entries.size() &&
        std::equal(this_entries.begin(), this_entries.end(), other_entries.begin(),
                   [](const MDD_Penalty& a, const MDD_Penalty& b) {
                     return a.mdd_id == b.mdd_id && a.penalty == b.penalty;
                   }))
      continue;

    std::cout << "compare_to_other_db: mismatch for mdd_id=" << mdd_id
               << " (other_db mdd_id=" << other_mdd_id << ")" << std::endl;

    size_t i = 0, j = 0;
    while (i < this_entries.size() && j < other_entries.size()) {
      const MDD_Penalty& a = this_entries[i];
      const MDD_Penalty& b = other_entries[j];
      if (a.mdd_id == b.mdd_id) {
        if (a.penalty != b.penalty)
          std::cout << "  penalty differs for other_db mdd_id=" << a.mdd_id
                     << ": this=" << (int)a.penalty << " other=" << (int)b.penalty << std::endl;
        i++;
        j++;
      } else if (a.mdd_id < b.mdd_id) {
        std::cout << "  extra in this: other_db mdd_id=" << a.mdd_id
                   << " penalty=" << (int)a.penalty << std::endl;
        i++;
      } else {
        std::cout << "  missing from this (extra in other): other_db mdd_id=" << b.mdd_id
                   << " penalty=" << (int)b.penalty << std::endl;
        j++;
      }
    }
    for (; i < this_entries.size(); i++)
      std::cout << "  extra in this: other_db mdd_id=" << this_entries[i].mdd_id
                 << " penalty=" << (int)this_entries[i].penalty << std::endl;
    for (; j < other_entries.size(); j++)
      std::cout << "  missing from this (extra in other): other_db mdd_id=" << other_entries[j].mdd_id
                 << " penalty=" << (int)other_entries[j].penalty << std::endl;

    return;
  }

  std::cout << "DBs are equal" << std::endl;
}