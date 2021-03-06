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

#ifndef MPD_DEV_GUI_POLYGON_SOUP_RENDERER_H_
#define MPD_DEV_GUI_POLYGON_SOUP_RENDERER_H_

#include <Eigen/Geometry>
#include "polygon_soup.h"

void drawPolygonSoup(const PolygonSoup& soup, const Eigen::Vector4d& i_base_color);

void drawPolygonSoup(const PolygonSoup& soup, const Eigen::Affine3d& transform, const Eigen::Vector4d& i_base_color);

#endif // MPD_DEV_GUI_POLYGON_SOUP_RENDERER_H_
