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

#include "gui/soft_body_renderer.h"

#include "GL/glew.h"
#include "SDL_opengl.h"

void renderSoftBody(const SoftBody& i_body, const Eigen::Vector4d& i_faces_color, const Eigen::Vector4d& i_edges_color, const Eigen::Vector4d& i_nodes_color, 
	bool i_render_faces, bool i_render_edges, bool i_render_nodes)
{
	const PolygonSoup& soft_base_geom  = i_body.base_geometry();
	const unsigned int nnodes = i_body.nb_nodes();
	const std::vector<Eigen::Vector3d>& nodes_pos = i_body.nodes_positions();
	const std::vector<Eigen::Vector3d>& nodes_normals = i_body.nodes_normals();
	
	if (i_render_faces)
	{
		glBegin(GL_TRIANGLES);
		// need to recompute triangle normals, as so far we do not want to do it in physics
		for (size_t i = 0 ; i < soft_base_geom.tris().size() ; ++i)
		{
			const Triangle& tri = soft_base_geom.tris()[i];
			const Eigen::Vector3d& v0 = nodes_pos[tri[0]];
			const Eigen::Vector3d& v1 = nodes_pos[tri[1]];
			const Eigen::Vector3d& v2 = nodes_pos[tri[2]];
			const Eigen::Vector3d& n0 = nodes_normals[tri[0]];
			const Eigen::Vector3d& n1 = nodes_normals[tri[1]];
			const Eigen::Vector3d& n2 = nodes_normals[tri[2]];
			const double lum0 = (2. + 1.3*n0.x() + 0.5*n0.y() + 0.8*n0.z()) / 4.;
			const double lum1 = (2. + 1.3*n1.x() + 0.5*n1.y() + 0.8*n1.z()) / 4.;
			const double lum2 = (2. + 1.3*n2.x() + 0.5*n2.y() + 0.8*n2.z()) / 4.;
			glColor4d(lum0 * i_faces_color[0], lum0 * i_faces_color[1], lum0 * i_faces_color[2], i_faces_color[3]);
			glVertex3dv(v0.data());
			glColor4d(lum1 * i_faces_color[0], lum1 * i_faces_color[1], lum1 * i_faces_color[2], i_faces_color[3]);
			glVertex3dv(v1.data());
			glColor4d(lum2 * i_faces_color[0], lum2 * i_faces_color[1], lum2 * i_faces_color[2], i_faces_color[3]);
			glVertex3dv(v2.data());

			//const Eigen::Vector3d n = (v1 - v0).cross(v2 - v0).normalized();
   //   const double lum = (2. + 1.3*n.x() + 0.5*n.y() + 0.8*n.z()) / 4.;
			//glColor4d(lum * i_faces_color[0], lum * i_faces_color[1], lum * i_faces_color[2], i_faces_color[3]);
			//glVertex3dv(v0.data());
			//glVertex3dv(v1.data());
			//glVertex3dv(v2.data());
		}
		glEnd();
	}
	if (i_render_edges)
	{
		// TODO
		//glBegin(GL_LINES);
		//for (size_t i = 0 ; i < i_coloured_line_array.size() ; ++i)
		//{

		//}
		//glEnd();
	}
	if (i_render_nodes)
	{
		glBegin(GL_POINTS);
		glPointSize(2.5);
		glColor4dv(i_nodes_color.data());
		for (size_t i = 0 ; i < nnodes ; ++i)
		{
			glVertex3dv(nodes_pos[i].data());
		}
		glEnd();
	}
}