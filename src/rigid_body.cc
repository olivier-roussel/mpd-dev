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

#include "rigid_body.h"

RigidBody::RigidBody(const PolygonSoup& i_soup, double i_mass, const Eigen::Affine3d& i_tranform):
	m_geometry(i_soup),
	m_mass(i_mass),
	m_transform(i_tranform)
	{}

RigidBody::~RigidBody()
{
}

const PolygonSoup& RigidBody::polygon_soup() const
{
	return m_geometry;
}

const Eigen::Affine3d& RigidBody::transform() const
{
	return m_transform;
}

void RigidBody::switchPolygonSoupAxis()
{
  m_geometry.switchYZAxis();
}

void RigidBody::invertPolygonSoupTriangles()
{
  m_geometry.invertTriangles();
}

double RigidBody::mass() const
{
	return m_mass;
}

void RigidBody::set_transform(const Eigen::Affine3d& i_transform)
{
	m_transform = i_transform;
}
