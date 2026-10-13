// Console UI (Norton Commander style, dark) for browsing and running evaluations.
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <cstring>
#include <cstdio>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

const std::string EVAL_DIR = "evaluations";

struct Progress {
  long done = 0;
  long total = 0;
  bool has_csv = false;
  bool running = false;
  int pid = 0;
  int last_exit = -1;  // -1: never finished a run from this UI
};

std::mutex g_mutex;
std::map<std::string, Progress> g_progress;  // guarded by g_mutex
std::map<std::string, pid_t> g_children;     // guarded by g_mutex
std::map<std::string, int> g_last_exit;      // guarded by g_mutex
std::vector<std::string> g_names;            // immutable after startup
std::atomic<bool> g_dirty{true};
std::atomic<bool> g_quit{false};
std::atomic<int> g_signal{0};
termios g_saved_term;
bool g_term_saved = false;

void on_signal(int sig)
{
  if (sig == SIGWINCH) {
    g_dirty = true;
  } else {
    g_signal = sig;
    g_quit = true;
  }
}

std::string json_path(const std::string& name)
{
  return EVAL_DIR + "/" + name + ".json";
}

std::string csv_path(const std::string& name)
{
  return EVAL_DIR + "/" + name + ".results.csv";
}

std::string read_file(const std::string& path)
{
  std::ifstream in(path);
  std::stringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

long json_int(const std::string& text, const std::string& key)
{
  std::smatch m;
  std::regex r("\"" + key + "\"\\s*:\\s*(\\d+)");
  if (std::regex_search(text, m, r)) return std::stol(m[1].str());
  return 0;
}

// Counting newlines is enough: one header line, one line per finished run.
long count_lines(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) return -1;
  long n = 0;
  char buf[4096];
  while (in.read(buf, sizeof buf) || in.gcount() > 0) {
    n += std::count(buf, buf + in.gcount(), '\n');
  }
  return n;
}

std::string eval_runner_path()
{
  char buf[4096];
  const auto len = readlink("/proc/self/exe", buf, sizeof buf - 1);
  if (len <= 0) return "./eval_runner";
  buf[len] = 0;
  return (fs::path(buf).parent_path() / "eval_runner").string();
}

void start_run(const std::string& name, bool recover)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_children.count(name)) return;
  const auto runner = eval_runner_path();
  const auto json = json_path(name);
  const pid_t pid = fork();
  if (pid < 0) return;
  if (pid == 0) {
    prctl(PR_SET_PDEATHSIG, SIGKILL);  // die with the UI even if it crashes
    const int null_fd = open("/dev/null", O_RDWR);
    if (null_fd >= 0) {
      dup2(null_fd, 0);
      dup2(null_fd, 1);
      dup2(null_fd, 2);
    }
    if (recover) {
      execl(runner.c_str(), runner.c_str(), "-recover", json.c_str(), (char*)nullptr);
    } else {
      execl(runner.c_str(), runner.c_str(), json.c_str(), (char*)nullptr);
    }
    _exit(127);
  }
  g_children[name] = pid;
  g_last_exit.erase(name);
  g_dirty = true;
}

void kill_children()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  for (auto& [name, pid] : g_children) kill(pid, SIGTERM);
  for (int i = 0; i < 50 && !g_children.empty(); ++i) {
    for (auto it = g_children.begin(); it != g_children.end();) {
      if (waitpid(it->second, nullptr, WNOHANG) != 0) {
        it = g_children.erase(it);
      } else {
        ++it;
      }
    }
    if (!g_children.empty()) usleep(100 * 1000);
  }
  for (auto& [name, pid] : g_children) {
    kill(pid, SIGKILL);
    waitpid(pid, nullptr, 0);
  }
  g_children.clear();
}

void monitor_loop()
{
  while (!g_quit) {
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      for (auto it = g_children.begin(); it != g_children.end();) {
        int status = 0;
        if (waitpid(it->second, &status, WNOHANG) > 0) {
          g_last_exit[it->first] =
              WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
          it = g_children.erase(it);
        } else {
          ++it;
        }
      }
      for (const auto& name : g_names) {
        Progress p;
        const auto text = read_file(json_path(name));
        p.total = json_int(text, "num_of_scenes") * json_int(text, "seeds_per_scene");
        const auto lines = count_lines(csv_path(name));
        p.has_csv = lines >= 0;
        p.done = std::max(0L, lines - 1);
        if (p.total > 0) p.done = std::min(p.done, p.total);
        auto c = g_children.find(name);
        p.running = c != g_children.end();
        p.pid = p.running ? c->second : 0;
        auto e = g_last_exit.find(name);
        p.last_exit = e == g_last_exit.end() ? -1 : e->second;
        g_progress[name] = p;
      }
    }
    g_dirty = true;
    for (int i = 0; i < 5 && !g_quit; ++i) usleep(100 * 1000);
  }
}

// ---------------------------------------------------------------- drawing

enum Style { NORMAL, BORDER, HIGHLIGHT, BAR, KEYS, RUNNING };

const char* sgr(int style)
{
  switch (style) {
    case BORDER: return "\x1b[0;36m";
    case HIGHLIGHT: return "\x1b[0;30;46m";
    case BAR: return "\x1b[0;32m";
    case KEYS: return "\x1b[0;1;37m";
    case RUNNING: return "\x1b[0;1;33m";
    default: return "\x1b[0m";
  }
}

struct Cell {
  std::string ch = " ";
  int style = NORMAL;
};

struct Canvas {
  int rows, cols;
  std::vector<Cell> cells;
  Canvas(int r, int c) : rows(r), cols(c), cells(r * c) {}

  void put(int r, int c, const std::string& ch, int style)
  {
    if (r < 0 || r >= rows || c < 0 || c >= cols) return;
    cells[r * cols + c] = {ch, style};
  }

  // text is clipped to [c, c_end)
  void text(int r, int c, std::string s, int style, int c_end)
  {
    for (char& ch : s) {
      if (ch == '\t') ch = ' ';
      if (ch < 32 || (unsigned char)ch >= 127) ch = '?';
    }
    for (size_t i = 0; i < s.size() && c + (int)i < c_end; ++i) {
      put(r, c + i, std::string(1, s[i]), style);
    }
  }

  void fill(int r, int c0, int c1, int style)
  {
    for (int c = c0; c < c1; ++c) put(r, c, " ", style);
  }

  std::string render() const
  {
    std::string out = "\x1b[H";
    int cur = -1;
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        const auto& cell = cells[r * cols + c];
        if (cell.style != cur) {
          out += sgr(cell.style);
          cur = cell.style;
        }
        out += cell.ch;
      }
      if (r + 1 < rows) out += "\r\n";
    }
    return out + "\x1b[0m";
  }
};

void draw(int selected, int& top)
{
  winsize ws{};
  ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws);
  const int R = std::max<int>(ws.ws_row, 10);
  const int C = std::max<int>(ws.ws_col, 40);
  Canvas cv(R, C);

  const int L = std::clamp(C / 4, 16, C - 20);  // vertical divider column
  const int sep_row = R - 3;                    // above the command line
  const int prog_sep = 3;                       // under the progress pane

  // borders
  for (int c = 0; c < C; ++c) {
    cv.put(0, c, "─", BORDER);
    cv.put(R - 1, c, "─", BORDER);
    cv.put(sep_row, c, "─", BORDER);
  }
  for (int c = L + 1; c < C - 1; ++c) cv.put(prog_sep, c, "─", BORDER);
  for (int r = 0; r < R; ++r) {
    cv.put(r, 0, "│", BORDER);
    cv.put(r, C - 1, "│", BORDER);
    if (r > 0 && r < sep_row) cv.put(r, L, "│", BORDER);
  }
  cv.put(0, 0, "┌", BORDER);
  cv.put(0, C - 1, "┐", BORDER);
  cv.put(R - 1, 0, "└", BORDER);
  cv.put(R - 1, C - 1, "┘", BORDER);
  cv.put(0, L, "┬", BORDER);
  cv.put(sep_row, 0, "├", BORDER);
  cv.put(sep_row, C - 1, "┤", BORDER);
  cv.put(sep_row, L, "┴", BORDER);
  cv.put(prog_sep, L, "├", BORDER);
  cv.put(prog_sep, C - 1, "┤", BORDER);
  cv.text(0, 2, " Evaluations ", NORMAL, L);
  cv.text(0, L + 2, " Content ", NORMAL, C - 1);

  // command pane
  cv.text(R - 2, 2, "F1", KEYS, C - 1);
  cv.text(R - 2, 5, "- Run | ", NORMAL, C - 1);
  cv.text(R - 2, 13, "F2", KEYS, C - 1);
  cv.text(R - 2, 16, "- Recover | ", NORMAL, C - 1);
  cv.text(R - 2, 28, "F3", KEYS, C - 1);
  cv.text(R - 2, 31, "- Stats", NORMAL, C - 1);
  cv.text(R - 2, std::max(40, C - 12), "q - Quit", NORMAL, C - 1);

  // eval list
  const int list_h = sep_row - 1;
  const int n = g_names.size();
  if (selected < top) top = selected;
  if (selected >= top + list_h) top = selected - list_h + 1;
  top = std::max(0, std::min(top, std::max(0, n - list_h)));

  Progress sel;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (int i = 0; i < list_h && top + i < n; ++i) {
      const auto& name = g_names[top + i];
      const bool running = g_progress[name].running;
      const int style = top + i == selected ? HIGHLIGHT : NORMAL;
      cv.fill(1 + i, 1, L, style);
      cv.text(1 + i, 1, (running ? "*" : " ") + name, style, L);
    }
    if (n > 0) sel = g_progress[g_names[selected]];
  }

  // progress pane
  const int px = L + 2, pe = C - 2;
  if (n == 0) {
    cv.text(1, px, "no evaluation configs found in " + EVAL_DIR + "/", NORMAL, pe);
    cv.put(R - 2, 0, "│", BORDER);
    cv.put(R - 2, C - 1, "│", BORDER);
    std::string out = "\x1b[?25l" + cv.render();
    (void)!write(STDOUT_FILENO, out.data(), out.size());
    return;
  }
  if (sel.running) {
    cv.text(1, px, "RUNNING in background (pid " + std::to_string(sel.pid) + ")", RUNNING, pe);
  } else if (sel.last_exit > 0) {
    cv.text(1, px, "last run exited with code " + std::to_string(sel.last_exit), NORMAL, pe);
  } else {
    cv.text(1, px, "idle", NORMAL, pe);
  }
  if (!sel.has_csv) {
    cv.text(2, px, "no results", NORMAL, pe);
  } else {
    const int pct = sel.total > 0 ? (int)(100 * sel.done / sel.total) : 0;
    const std::string label = " " + std::to_string(pct) + "%  (" +
                              std::to_string(sel.done) + "/" +
                              std::to_string(sel.total) + ")";
    const int bar_w = std::max(4, std::min(50, pe - px - 2 - (int)label.size()));
    const int filled = sel.total > 0 ? (int)(bar_w * sel.done / sel.total) : 0;
    std::string bar = "[" + std::string(filled, '#') + std::string(bar_w - filled, '.') + "]";
    cv.text(2, px, bar, BAR, pe);
    cv.text(2, px + bar.size(), label, NORMAL, pe);
  }

  // json pane
  std::istringstream json(read_file(json_path(g_names[selected])));
  std::string line;
  for (int r = prog_sep + 1; r < sep_row && std::getline(json, line); ++r) {
    cv.text(r, px, line, NORMAL, pe + 1);
  }

  const std::string out = "\x1b[?25l" + cv.render();
  (void)!write(STDOUT_FILENO, out.data(), out.size());
}

void restore_terminal()
{
  if (!g_term_saved) return;
  const char* leave = "\x1b[0m\x1b[?25h\x1b[?1049l";
  (void)!write(STDOUT_FILENO, leave, strlen(leave));
  tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_term);
  g_term_saved = false;
}

enum Key { NONE, UP, DOWN, F1, F2, F3, QUIT };

// Decodes one key from buf[pos..], advancing pos.
Key next_key(const std::string& buf, size_t& pos)
{
  const unsigned char c = buf[pos++];
  if (c == 'q' || c == 'Q' || c == 3) return QUIT;
  if (c != 27) return NONE;
  if (pos >= buf.size()) return NONE;
  const char intro = buf[pos];
  if (intro == 'O') {  // ESC O A / ESC O P ...
    ++pos;
    if (pos >= buf.size()) return NONE;
    switch (buf[pos++]) {
      case 'A': return UP;
      case 'B': return DOWN;
      case 'P': return F1;
      case 'Q': return F2;
      case 'R': return F3;
    }
    return NONE;
  }
  if (intro != '[') return NONE;
  ++pos;
  if (pos < buf.size() && buf[pos] == '[') {  // linux console: ESC [ [ A
    ++pos;
    if (pos >= buf.size()) return NONE;
    switch (buf[pos++]) {
      case 'A': return F1;
      case 'B': return F2;
      case 'C': return F3;
    }
    return NONE;
  }
  std::string params;
  while (pos < buf.size() && !(buf[pos] >= 0x40 && buf[pos] <= 0x7e)) {
    params += buf[pos++];
  }
  if (pos >= buf.size()) return NONE;
  const char fin = buf[pos++];
  if (fin == 'A') return UP;
  if (fin == 'B') return DOWN;
  if (fin == '~') {
    if (params == "11") return F1;
    if (params == "12") return F2;
    if (params == "13") return F3;
  }
  return NONE;
}

}  // namespace

int main()
{
  if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
    fprintf(stderr, "eval_ui requires an interactive terminal\n");
    return 1;
  }

  if (fs::is_directory(EVAL_DIR)) {
    for (const auto& entry : fs::directory_iterator(EVAL_DIR)) {
      if (entry.is_regular_file() && entry.path().extension() == ".json") {
        g_names.push_back(entry.path().stem().string());
      }
    }
    std::sort(g_names.begin(), g_names.end());
  }

  struct sigaction sa{};
  sa.sa_handler = on_signal;  // no SA_RESTART so poll() is interrupted
  sigemptyset(&sa.sa_mask);
  for (int sig : {SIGINT, SIGTERM, SIGHUP, SIGWINCH}) sigaction(sig, &sa, nullptr);

  tcgetattr(STDIN_FILENO, &g_saved_term);
  g_term_saved = true;
  termios raw = g_saved_term;
  cfmakeraw(&raw);
  tcsetattr(STDIN_FILENO, TCSANOW, &raw);
  (void)!write(STDOUT_FILENO, "\x1b[?1049h\x1b[2J", 12);

  std::thread monitor(monitor_loop);

  int selected = 0, top = 0;
  while (!g_quit) {
    if (g_dirty.exchange(false)) draw(selected, top);
    pollfd pfd{STDIN_FILENO, POLLIN, 0};
    if (poll(&pfd, 1, 200) <= 0) continue;
    char tmp[64];
    const auto n = read(STDIN_FILENO, tmp, sizeof tmp);
    if (n <= 0) {
      if (n == 0) break;
      continue;
    }
    const std::string buf(tmp, n);
    for (size_t pos = 0; pos < buf.size();) {
      switch (next_key(buf, pos)) {
        case UP: if (selected > 0) --selected; break;
        case DOWN: if (selected + 1 < (int)g_names.size()) ++selected; break;
        case F1: if (!g_names.empty()) start_run(g_names[selected], false); break;
        case F2: if (!g_names.empty()) start_run(g_names[selected], true); break;
        case QUIT: g_quit = true; break;
        default: break;
      }
    }
    g_dirty = true;
  }

  g_quit = true;
  monitor.join();
  kill_children();  // children first, then the terminal
  restore_terminal();
  return 0;
}
