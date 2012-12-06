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

#ifndef MPD_DEV_GUI_SOFT_BODY_RENDERER_H_
#define MPD_DEV_GUI_SOFT_BODY_RENDERER_H_

#include "mpd/soft_body.h"

void renderSoftBody(const SoftBody& i_body, const Eigen::Vector4d& i_faces_color, const Eigen::Vector4d& i_edges_color, const Eigen::Vector4d& i_nodes_color, 
	bool i_render_faces, bool i_render_edges, bool i_render_nodes);

#endif // MPD_DEV_GUI_SOFT_BODY_RENDERER_H_
