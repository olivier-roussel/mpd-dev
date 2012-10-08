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
#include "gui/gl_viewer.h"

int main()
{
  GLViewer viewer("mpd-dev");
  if (!viewer.init())
  {
    std::cout << "Could not initialize GL viewer." << std::endl;
    return -1;
  }

  while (!viewer.is_done())
  {
    viewer.processEvents();
    viewer.renderScene();
    viewer.handleGUI();
  }

  viewer.quit();

  return 0;
}
