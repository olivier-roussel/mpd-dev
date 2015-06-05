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

#ifndef MPD_DEV_SOFT_BODY_PARAMETERS_H_
#define MPD_DEV_SOFT_BODY_PARAMETERS_H_

#include <limits>
#include <math.h>

struct SoftBodyParameters
{
	/*
	* Physical attributes
	*/
	// We make them public as they are many, but this would need a special generic structure

	double k_ERP;				/**< Velocity correction factor [?] */
	double k_DP;				/**< Damping coefficient [0, 1]*/

	double k_PR;				/**< Pressure coefficient [-inf, +inf] */
	double k_VC;				/**< Volum conservation coefficient [0, +inf] */
	double k_DF;				/**< Dynamic friction coefficient [0, 1] (0 = slides, 1 = sticks) */
	double k_MT;				/**< Pose matching coefficient [0, 1]. */
	double k_CHR;				/**< Rigid contact hardness [0, 1] (0 = no penetration correction, 1 = full correction) */
	double k_KHR;				/**< Kinetic/static contact hardness [0, 1] (0 = no penetration correction, 1 = full correction) */
	double k_SHR;				/**< Soft contact hardness [0, 1] (0 = no penetration correction, 1 = full correction) */
	double k_AHR;				/**< Anchors (joints) hardness [0, 1] (0 = no drift correction, 1 = full correction) */

	int v_niters;				/**< Number of iterations for velocities solvers (if any) */
	int p_niters;				/**< Number of iterations for positions solvers (if any) */
	int d_niters;				/**< Number of iterations for drift solvers (if any) */

	/*
	* Material
	*/

	double k_LST;			/**< Linear stiffness coefficient [0, 1]. */
	double k_AST;			/**< Angular stiffness coefficient [0, 1]. */
	double k_VST;			/**< Volume stiffness coefficient [0, 1]. */


	/**
	*	Methods
	*/
	SoftBodyParameters();
	SoftBodyParameters(const SoftBodyParameters& other);

	//SoftBodyParameters& operator=(const SoftBodyParameters& i_other);
	
	bool isEqual(const SoftBodyParameters& i_other, double i_float_tolerance = std::numeric_limits<double>::epsilon()) const;

};

#endif // MPD_DEV_SOFT_BODY_PARAMETERS_H_
