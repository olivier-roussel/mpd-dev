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

#ifndef MPD_DEV_GUI_RENDER_HELPERS_H_
#define MPD_DEV_GUI_RENDER_HELPERS_H_

#include <Eigen/Core>
#include <vector>
#include <boost/tuple/tuple.hpp>

struct RenderingConfiguration
{
	// General rendering 
	bool render_world_referential;

	// Soft body rendering
	bool soft_render_faces;	// will requires triangle normals recomputation at each rendering 
	bool soft_render_edges;
	bool soft_render_nodes;
	
	// General physics rendering
	bool render_contact_points;	// TODO
	bool render_contact_forces;	// TODO

	// debugging purpose
	bool render_physics_from_engine; // use rendering from physics engine if available
};

void drawCylinder(float minx, float miny, float minz, float maxx, float maxy, float maxz, const Eigen::Vector4f& color_);

void renderReferential(const Eigen::Vector3d& pos, float len_factor, float width_factor);

void drawLineArray(const std::vector<boost::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> >& i_coloured_line_array);

void draw3dText(const Eigen::Vector3d& pos, const std::string& text, const Eigen::Vector3d& color, const double scale = 1.);

#endif // MPD_DEV_GUI_RENDER_HELPERS_H_
