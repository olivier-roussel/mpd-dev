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

#ifndef MPD_DEV_CONSTANTS_H_
#define MPD_DEV_CONSTANTS_H_

#include <Eigen/Core>

static const double kPi = 3.14159265358979323846;

/**
 * Transformation matrix from Z up axis to Y up axis (both direct & right handed)
 *         |  1  0  0 |
 * Z_2_Y = |  0  0  1 |
 *         |  0 -1  0 |
 */
static const double Z_2_Y_data[] = {1, 0, 0, 0, 0, -1, 0, 1, 0};
 
static const Eigen::Matrix3d Z_2_Y_Matrix(Z_2_Y_data); 

/**
 * Transformation matrix from Y up axis to Z up axis (both direct & right handed)
 *         |  1  0  0 |
 * Y_2_Z = |  0  0 -1 |
 *         |  0  1  0 |
 */ 
static const double Y_2_Z_data[] = {1, 0, 0, 0, 0, 1, 0, -1, 0};
 
static const Eigen::Matrix3d Y_2_Z_Matrix(Y_2_Z_data); 

#endif // MPD_DEV_CONSTANTS_H_
