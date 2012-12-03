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
#include "mpd/constants.h"

#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <fstream>
#include <iostream>
#include <sstream>

PolygonSoup::PolygonSoup() :
  verts_(), tris_(), normals_(), aabb_(), up_axis_(UA_Y_UP)
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

const AABB& PolygonSoup::aabb() const
{
  return aabb_;
}

void PolygonSoup::addVertex(const Eigen::Vector3d& i_vert)
{
	verts_.push_back(i_vert);
}

void PolygonSoup::addTriangle(const Triangle& i_tri)
{
	tris_.push_back(i_tri);
}

bool PolygonSoup::loadFromFile(const boost::filesystem::path& path)
{
  bool res = false;
  clear();
  if (boost::filesystem::exists(path) && boost::filesystem::is_regular_file(path))
  {
    std::string ext(path.extension().string());
    boost::to_upper(ext);
    if (ext == ".OBJ")
      res = loadFromObj(path);
//    else if (ext == "DAE")
//      loadFromCollada(filename, std::string(filename + ".trims"), std::string(fileName + ".nodes"));
  }

  // MPD viewer uses Z up axis, so switch up axis if needed
  if (res && up_axis_ == UA_Y_UP)
    switchYZAxis();

  return res;
}

bool PolygonSoup::loadFromObj(const boost::filesystem::path& filename)
{
  std::ifstream file;
  file.open(filename.string().c_str(), std::ifstream::in | std::ifstream::binary);
  while (file.good())
  {
    // parse row by row
    std::string line;
    getline(file, line);
    if (line.size() < 2 || line[0] == '#')
      continue;
    // vertex coords
    if (line[0] == 'v' && line[1] != 'n' && line[1] != 't')
    {
      std::stringstream ss(line.substr(1));
      double x,y,z;
      ss >> x >> y >> z;
      verts_.push_back(Eigen::Vector3d(x, y, z));
    }else if(line[0] == 'f')
    {
      // face
      // extract substring for each vertex of this face
      std::vector<unsigned int> vert_indexes;
      std::vector<std::string> tokens;
      std::istringstream iss(line.substr(1));
      std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
          std::back_inserter<std::vector<std::string> >(tokens));
      for (size_t i = 0 ; i < tokens.size() ; ++i)
      {
        std::replace(tokens[i].begin(), tokens[i].end(), '/', ' ');
        std::vector<std::string> vert_fields;
        std::istringstream iss_vert(tokens[i]);
        std::copy(std::istream_iterator<std::string>(iss_vert), std::istream_iterator<std::string>(),
          std::back_inserter<std::vector<std::string> >(vert_fields));
        if (vert_fields.empty())
          return false; // wrong file format
        const unsigned int vert_index = boost::lexical_cast<unsigned int>(vert_fields[0]) - 1; // !! indexes start at 1 in obj files
        vert_indexes.push_back(vert_index);
      }
      for (size_t i = 2 ; i < vert_indexes.size() ; ++i)
        tris_.push_back(boost::make_tuple(vert_indexes[0], vert_indexes[i-1], vert_indexes[i]));
    }
  }
  file.close();
  up_axis_ = UA_Y_UP; // the obj standard does not say anything about up axis, but seems Y up is commonly used
  computeAABB();
  computeTriangleNormals();
  return true;
}

void PolygonSoup::invertTriangles()
{
  for (size_t i = 0  ; i < tris_.size() ; ++i)
    std::swap(tris_[i].get<1>(), tris_[i].get<2>());
}

void PolygonSoup::clear()
{
  verts_.clear();
  tris_.clear();
  normals_.clear();
  aabb_.bmin = Eigen::Vector3d::Zero();
  aabb_.bmax = Eigen::Vector3d::Zero();
}

void PolygonSoup::switchYZAxis()
{
  if (up_axis_ == UA_Y_UP)
  {
    // goto Z vertical
    for (size_t i = 0 ; i < verts_.size() ; ++i)
      verts_[i] = Y_2_Z_Matrix * verts_[i];

    for (int i = 0 ; i < normals_.size() ; ++i)
      normals_[i] = Y_2_Z_Matrix * normals_[i];

    computeAABB();
    up_axis_ = UA_Z_UP;
  }
  else if (up_axis_ == UA_Z_UP) 
  {
    // goto Y vertical
    for (size_t i = 0 ; i < verts_.size() ; ++i)
      verts_[i] = Z_2_Y_Matrix * verts_[i];

    for (int i = 0 ; i < normals_.size() ; ++i)
      normals_[i] = Z_2_Y_Matrix * normals_[i];

    computeAABB();
    up_axis_ = UA_Y_UP;
  }else{
    std::cout << "PolygonSoup::switchYZAxis() : Unknown axis state" << std::endl;
  }
}

const PolygonSoup::UpAxis_t PolygonSoup::up_axis() const
{
  return up_axis_;
}
  
void PolygonSoup::computeAABB()
{
  if (!isEmpty())
  {
    aabb_.bmin = verts_[0];
    aabb_.bmax = verts_[0];
    for (size_t i = 1 ; i < verts_.size() ; ++i)
    {
      aabb_.bmin = aabb_.bmin.cwiseMin(verts_[i]);
      aabb_.bmax = aabb_.bmax.cwiseMax(verts_[i]);
    }
  }else{
    aabb_.bmin = Eigen::Vector3d::Zero();
    aabb_.bmax = Eigen::Vector3d::Zero();
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

