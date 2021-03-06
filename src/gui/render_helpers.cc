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

#include "render_helpers.h"

#include "GL/glew.h"
#include "SDL_opengl.h"

#include "constants.h"
#include "gui/ugly_font.h"

void drawCylinder(float minx, float miny, float minz, float maxx, float maxy, float maxz, const Eigen::Vector4f& color_)
{
  static const int NUM_SEG = 16;
  float dir[NUM_SEG*2];
  for (int i = 0; i < NUM_SEG; ++i)
  {
    const float a = (float)i/(float)NUM_SEG*(float)kPi*2;
    dir[i*2] = cosf(a);
    dir[i*2+1] = sinf(a);
  }

  const float cx = (maxx + minx)/2;
  const float cy = (maxy + miny)/2;
  const float rx = (maxx - minx)/2;
  const float ry = (maxy - miny)/2;

  glColor4fv(color_.data());

  glBegin(GL_QUADS);
  for (int i = 0, j=NUM_SEG-1; i < NUM_SEG; j=i++)
  {
    glVertex3f(cx+dir[j*2+0]*rx, cy+dir[j*2+1]*ry, minz);
    glVertex3f(cx+dir[i*2+0]*rx, cy+dir[i*2+1]*ry, minz);
    glVertex3f(cx+dir[i*2+0]*rx, cy+dir[i*2+1]*ry, maxz);
    glVertex3f(cx+dir[j*2+0]*rx, cy+dir[j*2+1]*ry, maxz);
  }

  // draw cylinder top cap
  for (int i = 0; i < NUM_SEG/2 -1 ; ++i) 
  {
    glVertex3f(cx+dir[i*2+0]*rx, cy+dir[i*2+1]*ry, maxz);
    glVertex3f(cx+dir[(i+1)*2+0]*rx, cy+dir[(i+1)*2+1]*ry, maxz);
    glVertex3f(cx+dir[(i+NUM_SEG/2)*2+0]*rx, cy+dir[(i+NUM_SEG/2)*2+1]*ry, maxz);
    glVertex3f(cx+dir[(i+NUM_SEG/2+1)*2+0]*rx, cy+dir[(i+NUM_SEG/2+1)*2+1]*ry, maxz);
  }
  // last quad of cylinder top
  glVertex3f(cx+dir[(NUM_SEG/2-1)*2+0]*rx, cy+dir[(NUM_SEG/2-1)*2+1]*ry, maxz);
  glVertex3f(cx+dir[(NUM_SEG/2)*2+0]*rx, cy+dir[(NUM_SEG/2)*2+1]*ry, maxz);
  glVertex3f(cx+dir[(NUM_SEG-1)*2+0]*rx, cy+dir[(NUM_SEG-1)*2+1]*ry, maxz);
  glVertex3f(cx+dir[(0)*2+0]*rx, cy+dir[(0)*2+1]*ry, maxz);

  glEnd();

}

void renderReferential(const Eigen::Vector3d& pos, float len_factor, float width_factor)
{
  glLineWidth(width_factor);
  glPushMatrix();
    glTranslatef(pos[0], pos[1], pos[2]);
    glScalef(len_factor, len_factor, len_factor);
    glBegin(GL_LINES);
    glColor3ub(255, 0, 0); //x axis
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(1.f, 0.f, 0.f);
    glColor3ub(0, 255, 0); //y axis
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 1.f, 0.f);
    glColor3ub(0, 0, 255); //z axis
    glVertex3f(0.f, 0.f, 0.f);
    glVertex3f(0.f, 0.f, 1.f);
    glEnd();
  glPopMatrix();
}

void draw3dText(const Eigen::Vector3d& pos, const std::string& text, const Eigen::Vector3d& color, const double scale)
{
	glPushMatrix();
	glColor3dv(color.data());
	glTranslated(pos.x(), pos.y(), pos.z());
	glScaled(scale, scale, scale);
	char* idx_str = new char[text.length() + 1];
	YsDrawUglyFont(text.c_str(), 1, 1);
	glPopMatrix();
	delete [] idx_str;
}


void drawLineArray(const std::vector<boost::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> >& i_coloured_line_array)
{
  glBegin(GL_LINES);
	for (size_t i = 0 ; i < i_coloured_line_array.size() ; ++i)
	{
		const boost::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>& line = i_coloured_line_array[i];
		glColor3dv(line.get<2>().data());
		glVertex3dv(line.get<0>().data());
		glVertex3dv(line.get<1>().data());
	}
	glEnd();
}