#include "mpd/algorithm.h"

#include <vector>

const std::string getMotionPlanningAlgorithmName(const MotionPlanningAlgorithms& algo)
{
  static std::vector<std::string> v_algo_names(MPA_ALGO_NAMES, MPA_ALGO_NAMES + MPA_NB_ALGOS);
  return v_algo_names[static_cast<size_t>(algo)];
}
