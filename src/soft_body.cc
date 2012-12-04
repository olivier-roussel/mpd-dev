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

#include "mpd/soft_body.h"

SoftBody::SoftBody(const PolygonSoup& i_stable_geom, double i_total_mass, const std::vector<double>& i_nodes_masses, const Eigen::Affine3d& i_tranform):
	m_stable_geom(i_stable_geom),
	m_total_mass(i_total_mass),
	m_nodes_masses(i_nodes_masses),
	m_transform(i_tranform),
	m_nb_nodes(i_nodes_masses.size())
{
	assert (m_stable_geom.verts().size() == m_nodes_masses.size() && "inconsistency between nodes and stable vertices");
	m_verts_deformations.resize(m_nb_nodes);
}

SoftBody::~SoftBody()
{
}

const PolygonSoup& SoftBody::polygon_soup() const
{
	return m_stable_geom;
}

const Eigen::Affine3d& SoftBody::transform() const
{
	return m_transform;
}

void SoftBody::switchPolygonSoupAxis()
{
  m_stable_geom.switchYZAxis();
}

void SoftBody::invertPolygonSoupTriangles()
{
  m_stable_geom.invertTriangles();
}

double SoftBody::mass() const
{
	return m_total_mass;
}

void SoftBody::set_transform(const Eigen::Affine3d& i_transform)
{
	m_transform = i_transform;
}

const std::vector<Eigen::Vector3d>& SoftBody::verts_deformations() const
{
	return m_verts_deformations;
}

std::vector<Eigen::Vector3d>& SoftBody::verts_deformations_mutable()
{
	return m_verts_deformations;
}

unsigned int SoftBody::nb_nodes() const
{
	return m_nb_nodes;
}
