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

#ifndef GUI_UTILS_H
#define GUI_UTILS_H

#include <sstream>
#include <string>
#include <Eigen/Core>

// Common helper functions

template<class T> inline T sqr(T a) { return a*a; }
template<class T> inline T clamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

Eigen::Vector4f int2Color(int i);

bool intersectRayAABB(const Eigen::Vector3f& ray_orig_, const Eigen::Vector3f& ray_dir_,
  const Eigen::Vector3f& bmin_, const Eigen::Vector3f& bmax_, float& tmin_, Eigen::Vector3f& q_);

//bool intersectRayTriangle(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_dir, 
//    const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, float* t);
//
//bool intersectRayTriangle(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir, 
//    const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double* t);

#endif // GUI_UTILS_H

// cmake:sourcegroup=Gui

