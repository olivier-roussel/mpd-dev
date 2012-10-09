#include "render_helpers.h"

#include "SDL_opengl.h"

void drawCylinder(float minx, float miny, float minz, float maxx, float maxy, float maxz, const Eigen::Vector4f& color_)
{
  static const int NUM_SEG = 16;
  float dir[NUM_SEG*2];
  for (int i = 0; i < NUM_SEG; ++i)
  {
    const float a = (float)i/(float)NUM_SEG*(float)M_PI*2;
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

