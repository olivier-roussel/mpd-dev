#ifndef MPD_DEV_ALGORITHM_H_
#define MPD_DEV_ALGORITHM_H_

#include <string>

enum MotionPlanningAlgorithms {
  MPA_KPIECE_OMPL = 0,  /**< Default implementation of KPIECE from OMPL. */
  MPA_NB_ALGOS
};

static const char* const MPA_ALGO_NAMES[] = {"KPIECE OMPL"};

const std::string getMotionPlanningAlgorithmName(const MotionPlanningAlgorithms& algo);

#endif // MPD_DEV_ALGORITHM_H_
