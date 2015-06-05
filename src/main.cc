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

#include <iostream>
#include "gui/mpd_viewer.h"
#include "mpd_controller.h"
#include "bullet_debug_drawer.h"

int main(int argc, char **argv)
{
  MPDController controller;

  MPDViewer viewer("mpd-dev", 1200, 960, 70, controller);

	controller.setPhysicsDebugDrawer(&viewer);

  viewer.run();

  viewer.join();

  return 0;
}
