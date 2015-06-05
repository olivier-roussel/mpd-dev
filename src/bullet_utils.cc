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

#include "bullet_utils.h"
#include "constants.h"

btVector3 toBtVector3(const Eigen::Vector3d& i_v)
{
  return btVector3(static_cast<btScalar>(i_v.x()), static_cast<btScalar>(i_v.y()), static_cast<btScalar>(i_v.z()) );
}

void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, btVector3* o_varray)
{
  for (size_t i = 0 ; i < i_varray.size() ; ++i)
    *(o_varray+i) = toBtVector3(i_varray[i]);
}

void toBtVector3Array(const std::vector<Eigen::Vector3d>& i_varray, const Eigen::Matrix3d& i_transform, btVector3* o_varray)
{
  for (size_t i = 0 ; i < i_varray.size() ; ++i)
    *(o_varray+i) = toBtVector3(i_transform * i_varray[i]);
}

Eigen::Vector3d toEVector3(const btVector3& v)
{
	return Eigen::Vector3d(v.x(), v.y(), v.z());
}

btQuaternion toBtQuaternion(const Eigen::Quaterniond& q)
{
  return btQuaternion(static_cast<btScalar>(q.x()),static_cast<btScalar>(q.y()),static_cast<btScalar>(q.z()),static_cast<btScalar>(q.w()));
}

Eigen::Quaterniond toEQuaternion(const btQuaternion& q)
{
  return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

btMatrix3x3 toBtMatrix3(const Eigen::Matrix3d& i_m)
{
	return btMatrix3x3(i_m(0, 0), i_m(0, 1), i_m(0, 2), 
		i_m(1, 0), i_m(1, 1), i_m(1, 2),
		i_m(2, 0), i_m(2, 1), i_m(2, 2));
}

btTransform toBtTransform(const Eigen::Affine3d& t)
{
	return btTransform(toBtMatrix3(t.rotation()), toBtVector3(t.translation()));
}

Eigen::Affine3d toETransform(const btTransform& t)
{
	Eigen::Affine3d e_t(Eigen::Affine3d::Identity());
	e_t.translate(toEVector3(t.getOrigin()));
	e_t.rotate(toEQuaternion(t.getRotation()));
	return e_t;
}
