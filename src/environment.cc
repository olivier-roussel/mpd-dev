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

#include "mpd/environment.h"

Environment::Environment()
{
}

Environment::~Environment()
{
}

const AABB& Environment::getAABB() const
{
  return polygon_soup_.aabb();
}

bool Environment::loadPolygonSoup(const boost::filesystem::path& path)
{
  return polygon_soup_.loadFromFile(path);
}

const PolygonSoup& Environment::polygon_soup() const
{
  return polygon_soup_;
}

void Environment::switchPolygonSoupAxis()
{
  polygon_soup_.switchYZAxis();
}

void Environment::invertPolygonSoupTriangles()
{
  polygon_soup_.invertTriangles();
}
