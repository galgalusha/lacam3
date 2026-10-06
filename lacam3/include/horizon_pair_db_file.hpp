#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

// Binary file format shared between HorizonPairDBGenerator::save_to_file()
// and HorizonPairDB::load_from_file(). The .mdd_db file layout is:
//   FileHeader | mdd_by_id section | mdd_id_by_v_g section | conflicts section | constrained_move_conflicts section

inline const std::string HORIZON_PAIR_DB_ROOT_FOLDER = "./mapf_db/";
inline const std::string HORIZON_PAIR_DB_FILE_EXTENSION = ".mdd_db";

// Terminates the per-frontier vertex list of a serialized MDD (see
// write_mdd/read_mdd in the generator/runtime classes).
inline constexpr uint8_t HORIZON_PAIR_DB_MDD_SENTINEL = 255;

// Header of the .mdd_db file: MDD count plus the byte offset of each section.
struct HorizonPairDBFileHeader {
  uint32_t num_mdds;
  uint64_t offset_mdd_by_id;
  uint64_t offset_mdd_id_by_v_g;
  uint64_t offset_conflicts;
  uint64_t offset_constrained_move_conflicts;
};

inline void print_horizon_pair_db_progress_bar(size_t done, size_t total) {
  const int bar_width = 40;
  float frac = total > 0 ? (float)done / (float)total : 1.0f;
  int filled = (int)(frac * bar_width);
  std::cout << "\r[";
  for (int k = 0; k < bar_width; ++k) std::cout << (k < filled ? '#' : '-');
  std::cout << "] " << std::fixed << std::setprecision(2) << (frac * 100.0f) << "%" << std::flush;
}
