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

#ifndef MPD_DEV_POLYGON_SOUP_H_
#define MPD_DEV_POLYGON_SOUP_H_

#include <Eigen/Core>
#include <vector>
#include <boost/filesystem/path.hpp>

#include "mpd/types.h"

class PolygonSoup {
public:
  
  PolygonSoup();
  ~PolygonSoup();


  const std::vector<Eigen::Vector3d>& verts() const;

  const std::vector<Triangle>& tris() const;

  const std::vector<Eigen::Vector3d>& normals() const;

  bool isEmpty() const;

  bool load(const std::string& filename);

  
private:
  std::vector<Eigen::Vector3d> verts_;      // Vertices array
  std::vector<Triangle> tris_;            // Triangles array
  std::vector<Eigen::Vector3d> normals_;    // Triangles normals
  Eigen::Vector3d bmin_, bmax_;             // Poly soup AABB (automatically updated)

  bool loadFromObj(const boost::filesystem::path& filename);
  void computeAABB();
  void computeTriangleNormals();

//  static const char * const kSupportedFileFormats;
  enum SupportedFileFormats {
    SFF_OBJ/*
    SFF_DAE*/
  };
};


#endif // MPD_DEV_POLYGON_SOUP_H_