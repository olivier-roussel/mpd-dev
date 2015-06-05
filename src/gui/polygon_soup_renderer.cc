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
#include "GL/glew.h"
#include "SDL_opengl.h"

void drawPolygonSoup(const PolygonSoup& soup, const Eigen::Vector4d& i_base_color)
{
  glBegin(GL_TRIANGLES);
  for (size_t i = 0 ; i < soup.tris().size() ; ++i)
  {
    const Triangle& t = soup.tris()[i];
    double lum = 0.85;
    if (soup.tris().size() == soup.normals().size())
      lum = (2. + 1.3*soup.normals()[i].x() + 0.5*soup.normals()[i].y() + 0.8*soup.normals()[i].z()) / 4.;
    glColor4d(lum * i_base_color[0], lum * i_base_color[1], lum * i_base_color[2], i_base_color[3]);
    glVertex3dv(soup.verts()[t[0]].data());
    glVertex3dv(soup.verts()[t[1]].data());
    glVertex3dv(soup.verts()[t[2]].data());
  }
  glEnd();
}

void drawPolygonSoup(const PolygonSoup& soup, const Eigen::Affine3d& transform, const Eigen::Vector4d& i_base_color)
{
	std::vector<Eigen::Vector3d> transformed_verts(soup.verts().size());
  for (size_t i = 0 ; i < soup.verts().size() ; ++i)
		transformed_verts[i] = transform * soup.verts()[i];

  glBegin(GL_TRIANGLES);
  for (size_t i = 0 ; i < soup.tris().size() ; ++i)
  {
    const Triangle& t = soup.tris()[i];
    double lum = 0.85;
    if (soup.tris().size() == soup.normals().size())
      lum = (2. + 1.3*soup.normals()[i].x() + 0.5*soup.normals()[i].y() + 0.8*soup.normals()[i].z()) / 4.;
    glColor4d(lum * i_base_color[0], lum * i_base_color[1], lum * i_base_color[2], i_base_color[3]);
    glVertex3dv(transformed_verts[t[0]].data());
    glVertex3dv(transformed_verts[t[1]].data());
    glVertex3dv(transformed_verts[t[2]].data());
  }
  glEnd();
}