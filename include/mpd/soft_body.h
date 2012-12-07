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

#ifndef MPD_DEV_SOFT_BODY_H_
#define MPD_DEV_SOFT_BODY_H_

#include <Eigen/Geometry>
#include "mpd/polygon_soup.h"
#include "mpd/soft_body_parameters.h"

class SoftBody
{
public:

	SoftBody(const PolygonSoup& i_base_geom, double i_total_mass, const std::vector<double>& i_nodes_masses, const Eigen::Affine3d& i_tranform);
	virtual ~SoftBody();

  void invertGeometryTriangles();

	const PolygonSoup& base_geometry() const;

	const Eigen::Affine3d& transform() const;

	void set_transform(const Eigen::Affine3d& i_transform);

	double mass() const;

	const std::vector<Eigen::Vector3d>& nodes_positions() const;

	std::vector<Eigen::Vector3d>& nodes_positions_mutable();

	const std::vector<Eigen::Vector3d>& nodes_normals() const;

	std::vector<Eigen::Vector3d>& nodes_normals_mutable();

	const SoftBodyParameters& parameters() const;

	void set_parameters(const SoftBodyParameters& i_params);

	//const std::vector<Eigen::Vector3d>& delta_nodes_position() const;

	//std::vector<Eigen::Vector3d>& delta_nodes_position_mutable();

	unsigned int nb_nodes() const;


protected:
	/*
	* Geometrical attributes
	*/
	std::vector<double> m_nodes_masses;
	double m_total_mass;
	PolygonSoup m_base_geometry;													// base body geometry (with n verts that will be deformable nodes)
	Eigen::Affine3d m_transform;													// TODO seems to be useless
	//std::vector<Eigen::Vector3d> m_delta_nodes_position;	// nodes deformations, i.e. difference between base positions and current positions
	std::vector<Eigen::Vector3d> m_nodes_positions;				// current nodes positions (size n)
	std::vector<Eigen::Vector3d> m_nodes_normals;					// current nodes normals	(size n)
	unsigned int m_nb_nodes;															// = n

	SoftBodyParameters m_params;
};

#endif // MPD_DEV_SOFT_BODY_H_
