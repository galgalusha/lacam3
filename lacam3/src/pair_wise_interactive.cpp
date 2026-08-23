#include "../include/pair_wise_db.hpp"
#include "../include/drawing.hpp"

#include <algorithm>
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>


// ── Terminal helpers ──────────────────────────────────────────────────────────

struct RawTerm {
  struct termios orig;
  RawTerm() {
    tcgetattr(STDIN_FILENO, &orig);
    struct termios raw = orig;
    raw.c_lflag &= ~(ECHO | ICANON);
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
  }
  ~RawTerm() { tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig); }
};

enum class Key { UP, DOWN, LEFT, RIGHT, ENTER, SPACE, ESC, QUIT, OTHER };

static Key read_key() {
  char c;
  if (read(STDIN_FILENO, &c, 1) <= 0) return Key::OTHER;
  if (c == '\x1b') {
    char seq[2];
    if (read(STDIN_FILENO, &seq[0], 1) <= 0) return Key::ESC;
    if (read(STDIN_FILENO, &seq[1], 1) <= 0) return Key::ESC;
    if (seq[0] == '[') {
      if (seq[1] == 'A') return Key::UP;
      if (seq[1] == 'B') return Key::DOWN;
      if (seq[1] == 'C') return Key::RIGHT;
      if (seq[1] == 'D') return Key::LEFT;
    }
    return Key::ESC;
  }
  if (c == '\r' || c == '\n') return Key::ENTER;
  if (c == ' ')                return Key::SPACE;
  if (c == 'q' || c == 'Q')   return Key::QUIT;
  return Key::OTHER;
}

static void cls() { std::cout << "\033[2J\033[H" << std::flush; }


// ── Entry point ───────────────────────────────────────────────────────────────

void PairWiseDB::test_interactive(Instance* ins) {
  // Load the entire pair-wise DB for this instance upfront
  std::cout << "Loading pair-wise DB...\n" << std::flush;
  // load_all2(ins);
  load_kernels(ins);

  const uint N = ins->N;

  // Save the original config; we temporarily override it for drawing
  const Config orig_starts = ins->starts;
  const Config orig_goals  = ins->goals;
  const uint   orig_N      = ins->N;

  // goal U-index → agent index (for cursor hit-testing)
  std::unordered_map<int, int> goal_u_to_agent;
  for (uint i = 0; i < N; ++i)
    if (orig_goals[i])
      goal_u_to_agent[orig_goals[i]->index] = (int)i;

  // Return the U-index of agent ag's goal, or -1
  auto goal_u_of = [&](int ag) -> int {
    return (ag >= 0 && orig_goals[ag]) ? orig_goals[ag]->index : -1;
  };

  // Find instance agents that have a non-zero dh with agent ag in pair_data
  auto find_paired = [&](int ag) -> std::vector<int> {
    if (ag < 0 || !orig_goals[ag]) return {};
    auto ig  = (uint16_t)orig_goals[ag]->id;
    auto is_ = orig_starts[ag] ? (uint16_t)orig_starts[ag]->id : (uint16_t)0;
    std::vector<int> result;
    for (uint j = 0; j < N; ++j) {
      if ((int)j == ag || !orig_goals[j]) continue;
      auto jg  = (uint16_t)orig_goals[j]->id;
      uint16_t lo = std::min(ig, jg), hi = std::max(ig, jg);
      if (!pair_data.count(to_goals_key(lo, hi))) continue;
      auto js_ = orig_starts[j] ? (uint16_t)orig_starts[j]->id : (uint16_t)0;
      // if (get_from_map(ig, jg, is_, js_) > 0)
      if (get(ag, j, orig_starts[ag], orig_starts[j]) > 0)
        result.push_back((int)j);
    }
    return result;
  };

  int cx = G->width / 2, cy = G->height / 2;
  int sel_agent = -1;
  std::vector<int> paired;

  RawTerm raw;

  while (true) {
    const int cursor_u = cy * G->width + cx;
    cls();

    if (sel_agent == -1) {
      // ── Cursor mode: show all instance goals, cursor, no starts ──────────
      draw_instance(ins, DrawExtra{cursor_u, /*goals_only=*/true});
    } else {
      // ── Selected mode: selected agent in white, paired agents in palette colors
      int sel_goal_u  = orig_goals[sel_agent]  ? orig_goals[sel_agent]->index  : -1;
      int sel_start_u = orig_starts[sel_agent] ? orig_starts[sel_agent]->index : -1;

      // Determine which paired agent (if any) the cursor hovers over
      int hover = -1;
      for (int j : paired) {
        if (goal_u_of(j) == cursor_u) { hover = j; break; }
      }

      // Show starts only for the hovered paired agent; others show goal only
      std::vector<std::pair<int,int>> agents;
      agents.reserve(paired.size());
      for (int j : paired) {
        int gu = orig_goals[j]  ? orig_goals[j]->index  : -1;
        int su = (j == hover && orig_starts[j]) ? orig_starts[j]->index : -1;
        agents.push_back({gu, su});
      }
      draw_map(G, {}, sel_goal_u, sel_start_u, agents, cursor_u);
    }

    std::cout << '\n';

    // ── Status line ───────────────────────────────────────────────────────
    if (sel_agent == -1) {
      auto it = goal_u_to_agent.find(cursor_u);
      if (it != goal_u_to_agent.end())
        std::cout << "On goal of agent " << it->second
                  << " (vertex " << orig_goals[it->second]->id << ")"
                  << "  Enter/Space: select\n";
      else if (G->U[cursor_u])
        std::cout << "Vertex " << G->U[cursor_u]->id
                  << "  (" << G->U[cursor_u]->x << ',' << G->U[cursor_u]->y << ")\n";
      else
        std::cout << "Obstacle\n";
      std::cout << "Arrows: move   Enter/Space on goal: select agent   q: quit\n";
    } else {
      // hover was already computed for drawing above; reuse it
      int hover = -1;
      for (int j : paired) {
        if (goal_u_of(j) == cursor_u) { hover = j; break; }
      }
      if (hover >= 0) {
        auto ig  = (uint16_t)orig_goals[sel_agent]->id;
        auto jg  = (uint16_t)orig_goals[hover]->id;
        auto is_ = orig_starts[sel_agent] ? (uint16_t)orig_starts[sel_agent]->id : (uint16_t)0;
        auto js_ = orig_starts[hover]     ? (uint16_t)orig_starts[hover]->id     : (uint16_t)0;
        // int dh   = (int)get_from_map(ig, jg, is_, js_);
        int dh = get(sel_agent, hover, orig_starts[sel_agent], orig_starts[hover]);
        std::cout << "Agent " << hover
                  << "  goal=" << jg << "  dh=" << dh << '\n';
      } else {
        std::cout << "Agent " << sel_agent
                  << " (goal=" << orig_goals[sel_agent]->id << ")"
                  << "  paired with " << paired.size() << " agents\n";
      }
      std::cout << "Arrows: move   ESC: deselect   q: quit\n";
    }
    std::cout << std::flush;

    Key k = read_key();

    // Movement always active
    if (k == Key::UP    && cy > 0)             --cy;
    if (k == Key::DOWN  && cy < G->height - 1) ++cy;
    if (k == Key::LEFT  && cx > 0)             --cx;
    if (k == Key::RIGHT && cx < G->width  - 1) ++cx;

    if (k == Key::QUIT) break;

    if (k == Key::ESC) {
      if (sel_agent >= 0) { sel_agent = -1; paired.clear(); }
      else break;
    }

    if (k == Key::ENTER || k == Key::SPACE) {
      auto it = goal_u_to_agent.find(cursor_u);
      if (it != goal_u_to_agent.end()) {
        int clicked = it->second;
        if (clicked == sel_agent) { sel_agent = -1; paired.clear(); }
        else                      { sel_agent = clicked; paired = find_paired(sel_agent); }
      }
    }
  }

  cls();
  std::cout << "Exiting interactive mode.\n";
}
