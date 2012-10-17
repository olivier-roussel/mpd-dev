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

#include "mpd/mpd_controller.h"

MPDController::MPDController()
  :env_(NULL)
{
}

MPDController::~MPDController()
{
  if (env_)
  {
    delete env_;
    env_ = NULL;
  }
}

const Environment& MPDController::environment() const
{
  assert(env_ != NULL && "environment is uninitialized.");

  return *env_;
}

bool MPDController::loadEnvironment(const boost::filesystem::path& path)
{
  if (!env_)
    env_ = new Environment();
  return env_->loadPolygonSoup(path);
}

void MPDController::switchEnvironmentAxis()
{
  if (env_)
    env_->switchPolygonSoupAxis();
}

void MPDController::invertEnvironmentTriangles()
{
  if (env_)
    env_->invertPolygonSoupTriangles();
}

bool MPDController::isEnvironmentSet() const
{
  return env_ != NULL;
}
