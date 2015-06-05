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
	inline BulletBaseArray(const BulletBaseArray& o) : data(o.data), size(o.size) {}
  inline ~BulletBaseArray() 
  {}
};

/**
 * Data type conversion helpers.
 */

/**
 *\fn btVector3 toBtVector3(const Eigen::Vector3d& v)
 *\brief Returns a Bullet vector3 from an Eigen vector3.
 */
btVector3 toBtVector3(const Eigen::Vector3d& v);

/**
 *\fn void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, btVector3* o_varray)
 *\brief Converts Eigen vector3 array to Bullet vector3 array.
 */
void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, btVector3* out);

/**
 *\fn void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, const Eigen::Matrix3d& i_transform, btVector3* o_varray)
 *\brief Converts Eigen vector3 array to Bullet vector3 array unsing given transformation matrix for each vector.
 */
void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, const Eigen::Matrix3d& i_transform, btVector3* o_varray);

/**
 *\fn Eigen::Vector3d toEVector3(const btVector3& v)
 *\brief Returns an Eigen vector3 from a Bullet vector3.
 */
Eigen::Vector3d toEVector3(const btVector3& v);

btMatrix3x3 toBtMatrix3(const Eigen::Matrix3d& i_m);

btQuaternion toBtQuaternion(const Eigen::Quaterniond& q);

Eigen::Quaterniond toEQuaternion(const btQuaternion& q);

btTransform toBtTransform(const Eigen::Affine3d& t);

Eigen::Affine3d toETransform(const btTransform& t);

#endif // MPD_DEV_BULLET_UTILS_H_

