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

btVector3 eig_v3_to_bt_v3(const Eigen::Vector3d& v)
{
  return btVector3(v.x(), v.y(), v.z());
}

void eig_v3_to_bt_v3_array(const std::vector<Eigen::Vector3d>& in, btVector3* out)
{
  for (size_t i = 0 ; i < in.size() ; ++i)
    *(out+i) = eig_v3_to_bt_v3(in[i]);
}
