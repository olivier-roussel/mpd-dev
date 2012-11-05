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
#include "btVector3.h"
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

/**
 * Data type conversion helpers.
 */

btVector3 eig_v3_to_bt_v3(const Eigen::Vector3d& v);

void eig_v3_to_bt_v3_array(const std::vector<Eigen::Vector3d>& in, btVector3& out);

#endif // MPD_DEV_BULLET_UTILS_H_

