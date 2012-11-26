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

#include "mpd/bullet_utils.h"

namespace e2bt
{

btVector3 toBtVector3(const Eigen::Vector3d& v)
{
  return btVector3(static_cast<btScalar>(v.x()), static_cast<btScalar>(v.y()), static_cast<btScalar>(v.z()) );
}

void toBtVector3Array(const std::vector<Eigen::Vector3d>& in, btVector3* out)
{
  for (size_t i = 0 ; i < in.size() ; ++i)
    *(out+i) = toBtVector3(in[i]);
}

btQuaternion toBtQuaternion(const Eigen::Quaterniond& q)
{
  return btQuaternion(static_cast<btScalar>(q.x()),static_cast<btScalar>(q.y()),static_cast<btScalar>(q.z()),static_cast<btScalar>(q.w()));
}

Eigen::Quaterniond toEQuaternion(const btQuaternion& q)
{
  return Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
}

btTransform toBtTransform(const Eigen::Affine3d& t)
{
  // TODO !!
  return btTransform();
}

} // namespace e2bt
