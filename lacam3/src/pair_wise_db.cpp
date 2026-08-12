#include "../include/pair_wise_db.hpp"

#include <fstream>
#include <stdexcept>

void append_entries(const std::string& path, const std::vector<PairEntry>& entries) {
  if (entries.empty()) return;
  std::ofstream f(path, std::ios::binary | std::ios::app);
  if (!f) throw std::runtime_error("Cannot open file: " + path);
  // PairEntry is a POD-like struct; write raw bytes for cheap append
  f.write(reinterpret_cast<const char*>(entries.data()),
          static_cast<std::streamsize>(entries.size() * sizeof(PairEntry)));
}

std::string serialize_for_rocksdb(const std::vector<PairEntry>& entries) {
  std::ostringstream os;
  {
    cereal::BinaryOutputArchive archive(os);
    archive(entries);
  }
  return os.str();
}
