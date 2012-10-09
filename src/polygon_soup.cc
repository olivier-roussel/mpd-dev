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

#include "mpd/polygon_soup.h"

#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <fstream>
#include <iostream>

PolygonSoup::PolygonSoup()
{}

PolygonSoup::~PolygonSoup()
{}


const std::vector<Eigen::Vector3d>& PolygonSoup::verts() const
{
  return verts_;
}

const std::vector<Triangle>& PolygonSoup::tris() const
{
  return tris_;
}

const std::vector<Eigen::Vector3d>& PolygonSoup::normals() const
{
  return normals_;
}

bool PolygonSoup::isEmpty() const
{
  return verts_.empty();
}

bool PolygonSoup::load(const std::string& filename)
{
  boost::filesystem::path path(filename);
  if (boost::filesystem::exists(path) && boost::filesystem::is_regular_file(path))
  {
    std::string ext(path.extension().string());
    boost::to_upper(ext);
    //m_name = path.stem().string();
    if (ext == "OBJ")
      return loadFromObj(path);
//    else if (ext == "DAE")
//      loadFromCollada(filename, std::string(filename + ".trims"), std::string(fileName + ".nodes"));
    else
      return false;
    computeAABB();
    computeTriangleNormals();
  }else
    return false;
}

bool PolygonSoup::loadFromObj(const boost::filesystem::path& filename)
{
  std::ifstream file;
  file.open(filename.string().c_str(), std::ifstream::in | std::ifstream::binary);
  while (file.good())
  {
    std::string line;
    getline(file, line);
    std::cout << line << std::endl; // debug
  }
  file.close();
}

void PolygonSoup::computeAABB()
{
  if (!isEmpty())
  {
    bmin_ = verts_[0];
    bmax_ = verts_[0];
    for (size_t i = 1 ; i < verts_.size() ; ++i)
    {
      bmin_ = bmin_.cwiseMin(verts_[i]);
      bmax_ = bmax_.cwiseMax(verts_[i]);
    }
  }
}

void PolygonSoup::computeTriangleNormals()
{
  normals_.clear();
  normals_.reserve(tris_.size());
  for (size_t i = 0 ; i < tris_.size() ; ++i)
  {
    const Eigen::Vector3d& v0 = verts_[tris_[i].get<0>()];
    const Eigen::Vector3d& v1 = verts_[tris_[i].get<1>()];
    const Eigen::Vector3d& v2 = verts_[tris_[i].get<2>()];
    const Eigen::Vector3d e0(v1 - v0), e1(v2 - v0);
    const Eigen::Vector3d n = e0.cross(e1).normalized();
    normals_.push_back(n);
  }
}

