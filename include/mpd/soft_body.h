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

// TODO still a lot to complete here

class SoftBody
{
public:
	SoftBody(const PolygonSoup& i_stable_geom, double i_total_mass, const std::vector<double>& i_nodes_masses, const Eigen::Affine3d& i_tranform);
	virtual ~SoftBody();

	void switchPolygonSoupAxis();

  void invertPolygonSoupTriangles();

	const PolygonSoup& polygon_soup() const;

	const Eigen::Affine3d& transform() const;

	void set_transform(const Eigen::Affine3d& i_transform);

	double mass() const;

	const std::vector<Eigen::Vector3d>& verts_deformations() const;

	std::vector<Eigen::Vector3d>& verts_deformations_mutable();

	unsigned int nb_nodes() const;

protected:
	std::vector<double> m_nodes_masses;
	double m_total_mass;
	PolygonSoup m_stable_geom;	
	Eigen::Affine3d m_transform;
	std::vector<Eigen::Vector3d> m_verts_deformations;
	unsigned int m_nb_nodes;
};

#endif // MPD_DEV_SOFT_BODY_H_
