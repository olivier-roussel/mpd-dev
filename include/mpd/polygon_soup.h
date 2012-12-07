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

class PolygonSoup 
{
public:
  
  enum SupportedFileFormats_t {
    SFF_OBJ = 0,
    /*SFF_DAE,*/
    SFF_NB_SUPPORTED_FORMATS
  };

  enum UpAxis_t {
    UA_Y_UP,
    UA_Z_UP
  };

	/**
	* Constructors & destructors
	*/

  PolygonSoup();
  ~PolygonSoup();

	/**
	* Modifiers
	*/

	/**
	* \post Must call computeAABB() once all vertices have been added.
	*/
	void addVertex(const Eigen::Vector3d& i_vert);

	/**
	* \post Must call computeTriangleNormals() once all triangles have been added.
	*/
	void addTriangle(const Triangle& i_tri);

  bool loadFromFile(const boost::filesystem::path& path);

  void clear();

  void computeAABB();

  void computeTriangleNormals();

  void invertTriangles();

  void switchYZAxis();

	/**
	* Accessors
	*/
	
  const std::vector<Eigen::Vector3d>& verts() const;

  const std::vector<Triangle>& tris() const;

  const std::vector<Eigen::Vector3d>& normals() const;

  const UpAxis_t up_axis() const;

  bool isEmpty() const;

  const Eigen::Vector3d& aabbmin() const;

  const Eigen::Vector3d& aabbmax() const;

	/**
	* Static methods
	*/

	static PolygonSoup createBox(double size);
  
private:
  std::vector<Eigen::Vector3d> verts_;      // Vertices array
  std::vector<Triangle> tris_;            // Triangles array
  std::vector<Eigen::Vector3d> normals_;    // Triangles normals
  Eigen::Vector3d aabb_min_, aabb_max_;             // Poly soup AABB (automatically updated)
  UpAxis_t up_axis_;

  bool loadFromObj(const boost::filesystem::path& filename);


//  static const char * const kSupportedFileFormats;
};


#endif // MPD_DEV_POLYGON_SOUP_H_
