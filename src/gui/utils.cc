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

#include "gui/utils.h"

#include <float.h>

bool intersectRayAABB(const Eigen::Vector3f& ray_orig_, const Eigen::Vector3f& ray_dir_,
  const Eigen::Vector3f& bmin_, const Eigen::Vector3f& bmax_, float& tmin_, Eigen::Vector3f& q_)
{
  static const float EPS = 1e-6f;
  tmin_ = 0.f;
  float tmax = FLT_MAX;

  for (int i = 0; i < 3 ; ++i)
  {
    if (fabs(ray_dir_[i]) < EPS)
    {
      // ray is parallel to slab: no hit if origin not within the slab
      if (ray_orig_[i] < bmin_[i] || ray_orig_[i] > bmax_[i])
        return false;
    }else{
      // compute intersection t value of ray with near and far plane of slab
      const float ood = 1.f / ray_dir_[i];
      float t1 = (bmin_[i] - ray_orig_[i]) * ood;
      float t2 = (bmax_[i] - ray_orig_[i]) * ood;
      // make t1 be intersection with near plan, t2 with far plane
      if (t1 > t2)
        std::swap(t1, t2);
      // compute the intersection of slab intersection intervals
      if (t1 > tmin_)
        tmin_ = t1;
      if (t2 < tmax)
        tmax = t2;
      // exit with no collision as soon as slab intersection becomes empty
      if (tmin_ > tmax)
        return false;
    }
  }

  // ray intersects all 3 slabs, return point q and intersection t value (tmin)
  q_ = ray_orig_ + ray_dir_ * tmin_;
  return true;
}

Eigen::Vector4f int2Color(int i)
{
  Eigen::Vector4f col;
  int r = (i & 1) + ((i & (1 << 3)) >> 3) * 2 + 1;
  int g = ((i & (1 << 1)) >> 1) + ((i & (1 << 4)) >> 4) * 2 + 1;
  int b = ((i & (1 << 2)) >> 2) + ((i & (1 << 5)) >> 5) * 2 + 1;
  col[0] = 1.f - static_cast<float>(r) * 63.0f/255.0f;
  col[1] = 1.f - static_cast<float>(g) * 63.0f/255.0f;
  col[2] = 1.f - static_cast<float>(b) * 63.0f/255.0f;
  col[3] = 1.f;
  return col;
}

//bool intersectRayTriangle(const Eigen::Vector3f& ray_origin, const Eigen::Vector3f& ray_dir, const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c, float* t)
//{
//  double t_d;
//  const bool res = intersectRayTriangle(ray_origin.cast<double>(), ray_dir.cast<double>(), a.cast<double>(), b.cast<double>(), c.cast<double>(), &t_d);
//  *t = t_d;
//  return res;
//}
//
//bool intersectRayTriangle(const Eigen::Vector3d& ray_origin, const Eigen::Vector3d& ray_dir, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double* t)
//{
//  static const double EPSILON = 1e-6;
//  const Eigen::Vector3d ab(b - a);
//  const Eigen::Vector3d ac(c - a);
//  const Eigen::Vector3d n(ab.cross(ac));
//
//  double d = ray_dir.dot(n);
//  if(d < -EPSILON)
//  {
//    d = -d;
//    const Eigen::Vector3d ao(ray_origin - a);
//    const double tt = ao.dot(n);
//    if(tt < -EPSILON) 
//      return false;
//
//    const Eigen::Vector3d e(ray_dir.cross(ao));
//    const double v = -ac.dot(e);
//
//    if( v < -EPSILON || v > d + EPSILON)
//      return false;
//
//    const double w = ab.dot(e);
//    if(w < -EPSILON || v + w > d + EPSILON) 
//      return false;
//
//    *t = tt / d;
//    return true;
//  }
//  else if(d > EPSILON)
//  {
//    const Eigen::Vector3d oa(a - ray_origin);
//    const double tt = oa.dot(n);
//
//    if(tt < -EPSILON)
//      return false;
//
//    const Eigen::Vector3d e(ray_dir.cross(oa));
//    const double v = -ac.dot(e);
//    if( v < -EPSILON || v > d + EPSILON) 
//      return false;
//
//    const double w = ab.dot(e);
//    if(w < -EPSILON || v + w > d + EPSILON) 
//      return false;
//
//    *t = tt / d;
//    return true;
//  }
//  else 
//    return false;
//}

// cmake:sourcegroup=Gui
