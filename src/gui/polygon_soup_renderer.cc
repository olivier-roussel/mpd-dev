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

#include "gui/polygon_soup_renderer.h"

#include "SDL.h"
#include "SDL_opengl.h"

void renderPolygonSoup(const PolygonSoup& soup)
{
  glBegin(GL_TRIANGLES);
  for (size_t i = 0 ; i < soup.tris().size() ; ++i)
  {
    const Triangle& t = soup.tris()[i];
    double lum = 0.85;
    if (soup.tris().size() == soup.normals().size())
      lum = (2. + 1.3*soup.normals()[i].x() + 0.5*soup.normals()[i].y() + 0.8*soup.normals()[i].z()) / 4.;
    glColor3d(lum, lum, lum);
    glVertex3dv(soup.verts()[t.get<0>()].data());
    glVertex3dv(soup.verts()[t.get<1>()].data());
    glVertex3dv(soup.verts()[t.get<2>()].data());
  }
  glEnd();
}
