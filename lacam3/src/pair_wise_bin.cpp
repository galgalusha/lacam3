#include "../include/pair_wise_bin.hpp"

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

void append_entries(const std::string& path, const std::vector<PairEntry>& entries) {
  if (entries.empty()) return;
  std::ofstream f(path, std::ios::binary | std::ios::app);
  if (!f) throw std::runtime_error("Cannot open file: " + path);
  // PairEntry is a POD-like struct; write raw bytes for cheap append
  f.write(reinterpret_cast<const char*>(entries.data()),
          static_cast<std::streamsize>(entries.size() * sizeof(PairEntry)));
}

void to_csv(const std::string& bin_path, const std::string& csv_path) {
  std::ifstream in(bin_path, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot open file: " + bin_path);
  std::ofstream out(csv_path);
  if (!out) throw std::runtime_error("Cannot open file: " + csv_path);
  out << "i_start,j_start,dh\n";
  PairEntry e;
  while (in.read(reinterpret_cast<char*>(&e), sizeof(PairEntry))) {
    out << e.i_start << ',' << e.j_start << ',' << e.dh << '\n';
  }
}

static auto entry_key(const PairEntry& e) {
  return to_key(e.i_start, e.j_start);
}

static std::vector<PairEntry> read_all(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return {};
  std::vector<PairEntry> v;
  PairEntry e;
  while (f.read(reinterpret_cast<char*>(&e), sizeof(PairEntry)))
    v.push_back(e);
  return v;
}

void merge_goal_folder(const std::string& base_dir, const std::string& goal_dir) {
  std::vector<std::filesystem::directory_entry> entries;
  for (const auto& e : std::filesystem::directory_iterator(goal_dir))
    if (e.is_regular_file()) entries.push_back(e);

  const size_t total = entries.size();
  size_t done        = 0;

  for (const auto& goal_entry : entries) {
    if (!goal_entry.is_regular_file()) continue;
    const std::string filename = goal_entry.path().filename().string();
    const std::string base_path = base_dir + "/" + filename;
    const std::string goal_path = goal_entry.path().string();
    const std::string tmp_path  = base_path + ".tmp";

    const auto base = read_all(base_path); // may be empty if file doesn't exist yet
    const auto goal = read_all(goal_path);

    // Merge two sorted sequences; goal overrides base on equal key
    std::ofstream out(tmp_path, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot open file: " + tmp_path);

    size_t bi = 0, gi = 0;
    while (bi < base.size() && gi < goal.size()) {
      auto bk = entry_key(base[bi]);
      auto gk = entry_key(goal[gi]);
      if (bk < gk) {
        out.write(reinterpret_cast<const char*>(&base[bi++]), sizeof(PairEntry));
      } else if (gk < bk) {
        out.write(reinterpret_cast<const char*>(&goal[gi++]), sizeof(PairEntry));
      } else {
        // equal key: goal overrides base
        out.write(reinterpret_cast<const char*>(&goal[gi++]), sizeof(PairEntry));
        ++bi;
      }
    }
    while (bi < base.size())
      out.write(reinterpret_cast<const char*>(&base[bi++]), sizeof(PairEntry));
    while (gi < goal.size())
      out.write(reinterpret_cast<const char*>(&goal[gi++]), sizeof(PairEntry));
    out.close();

    std::filesystem::rename(tmp_path, base_path);

    ++done;
    const int pct  = static_cast<int>(done * 100 / total);
    const int fill = pct / 2; // 50-char bar
    std::fprintf(stderr, "\r  merge_goal_folder [%-50s] %3d%%",
                 std::string(fill, '#').c_str(), pct);
    std::fflush(stderr);
  }
  std::fprintf(stderr, "\n");
}
