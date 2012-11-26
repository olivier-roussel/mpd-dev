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

#ifndef MPD_DEV_BULLET_UTILS_H_
#define MPD_DEV_BULLET_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include <vector>

/**
 * \struct Classic static array for bullet usage.
 */
template <class X>
struct BulletBaseArray {
  X* data;
  size_t size;

  inline BulletBaseArray() : data(NULL), size(0) {}
  inline ~BulletBaseArray()
  {
    if (data)
      delete[] data;
  }
};

namespace e2bt
{
/**
 * Data type conversion helpers.
 */

btVector3 toBtVector3(const Eigen::Vector3d& v);

void toBtVector3Array(const std::vector<Eigen::Vector3d>& in, btVector3* out);

btQuaternion toBtQuaternion(const Eigen::Quaterniond& q);

Eigen::Quaterniond toEQuaternion(const btQuaternion& q);

btTransform toBtTransform(const Eigen::Affine3d& t);

} // namespace e2bt

#endif // MPD_DEV_BULLET_UTILS_H_

