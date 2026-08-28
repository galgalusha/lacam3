#include "../include/lacam.hpp"
#include "../include/restarter.hpp"

Solution solve(const Instance &ins, int verbose, const Deadline *deadline,
               int seed)
{
  info(1, verbose, deadline, "pre-processing");
  auto planner = Planner(&ins, verbose, deadline, seed);
  planner.restarter = new SPD();
  return planner.solve();
}
