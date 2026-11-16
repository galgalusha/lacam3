#include <instance.hpp>

#include <set>

int main(int argc, char *argv[])
{
  if (argc != 4) return 1;
  const int count = std::stoi(argv[3]);
  const Instance instance(argv[1], argv[2], count);
  const std::set<Vertex *> starts(instance.starts.begin(), instance.starts.end());
  const std::set<Vertex *> goals(instance.goals.begin(), instance.goals.end());
  const bool valid = instance.is_valid() && starts.size() == instance.N &&
                     goals.size() == instance.N;
  delete instance.G;
  return valid ? 0 : 1;
}
