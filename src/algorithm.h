/**
* Copyright (c) 2012 CNRS
* Author: Olivier Roussel
*
* This file is part of the MPD-dev package.
* MPD-dev is free software: you can redistribute it
* and/or modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation, either version
* 3 of the License, or (at your option) any later version.
*
* MPD-dev is distributed in the hope that it will be
* useful, but WITHOUT ANY WARRANTY; without even the implied warranty
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* General Lesser Public License for more details.  You should have
* received a copy of the GNU Lesser General Public License along with
* MPD-dev.  If not, see
* <http://www.gnu.org/licenses/>.
**/

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
