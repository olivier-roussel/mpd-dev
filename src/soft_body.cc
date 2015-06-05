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

#include "soft_body.h"

SoftBody::SoftBody(const PolygonSoup& i_base_geometry, double i_total_mass, const std::vector<double>& i_nodes_masses, const Eigen::Affine3d& i_tranform):
	m_base_geometry(i_base_geometry),
	m_total_mass(i_total_mass),
	m_nodes_masses(i_nodes_masses),
	m_transform(i_tranform),
	m_nb_nodes(i_nodes_masses.size()),
	m_params()
{
	assert (m_base_geometry.verts().size() == m_nodes_masses.size() && "inconsistency between nodes and stable vertices");
	//m_delta_nodes_position.resize(m_nb_nodes);
	m_nodes_positions.resize(m_nb_nodes);
	m_nodes_normals.resize(m_nb_nodes);
	
}

SoftBody::~SoftBody()
{
}

const PolygonSoup& SoftBody::base_geometry() const
{
	return m_base_geometry;
}

const Eigen::Affine3d& SoftBody::transform() const
{
	return m_transform;
}

void SoftBody::invertGeometryTriangles()
{
  m_base_geometry.invertTriangles();
}

double SoftBody::mass() const
{
	return m_total_mass;
}

void SoftBody::set_transform(const Eigen::Affine3d& i_transform)
{
	m_transform = i_transform;
}

const std::vector<Eigen::Vector3d>& SoftBody::nodes_positions() const
{
	return m_nodes_positions;
}

std::vector<Eigen::Vector3d>& SoftBody::nodes_positions_mutable()
{
	return m_nodes_positions;
}

const std::vector<Eigen::Vector3d>& SoftBody::nodes_normals() const
{
	return m_nodes_normals;
}

std::vector<Eigen::Vector3d>& SoftBody::nodes_normals_mutable()
{
	return m_nodes_normals;
}

unsigned int SoftBody::nb_nodes() const
{
	return m_nb_nodes;
}

const SoftBodyParameters& SoftBody::parameters() const
{
	return m_params;
}

void SoftBody::set_parameters(const SoftBodyParameters& i_params)
{
	m_params = i_params;
}

// unused so far
//const std::vector<Eigen::Vector3d>& SoftBody::delta_nodes_position() const
//{
//	return m_delta_nodes_position;
//}
//
//std::vector<Eigen::Vector3d>& SoftBody::delta_nodes_position_mutable()
//{
//	return m_delta_nodes_position;
//}