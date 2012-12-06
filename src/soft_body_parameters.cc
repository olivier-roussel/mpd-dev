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

#include "mpd/soft_body_parameters.h"

SoftBodyParameters::SoftBodyParameters() :
k_ERP(1.),
	k_DP(0.),
	k_PR(0.),
	k_VC(0.),
	k_DF(0.2),
	k_MT(0.),
	k_CHR(1.),
	k_KHR(0.1),
	k_SHR(1.),
	k_AHR(0.7),
	v_niters(0),
	p_niters(1),
	d_niters(0),
	k_LST(1.),
	k_AST(1.),
	k_VST(1.)
{}

SoftBodyParameters::SoftBodyParameters(const SoftBodyParameters& other) :
k_ERP(other.k_ERP),
k_DP(other.k_DP),
k_PR(other.k_PR),
k_VC(other.k_VC),
k_DF(other.k_DF),
k_MT(other.k_MT),
k_CHR(other.k_CHR),
k_KHR(other.k_KHR),
k_SHR(other.k_SHR),
k_AHR(other.k_AHR),
v_niters(other.v_niters),
p_niters(other.p_niters),
d_niters(other.d_niters),
k_LST(other.k_LST),
k_AST(other.k_AST),
k_VST(other.k_VST)
{}

bool SoftBodyParameters::isEqual(const SoftBodyParameters& i_other, double i_float_tolerance) const
{
	if (abs(k_ERP - i_other.k_ERP) > i_float_tolerance ||
		abs(k_DP - i_other.k_DP) > i_float_tolerance ||
		abs(k_PR - i_other.k_PR) > i_float_tolerance ||
		abs(k_VC - i_other.k_VC) > i_float_tolerance ||
		abs(k_DF - i_other.k_DF) > i_float_tolerance ||
		abs(k_MT - i_other.k_MT) > i_float_tolerance ||
		abs(k_CHR - i_other.k_CHR) > i_float_tolerance ||
		abs(k_KHR - i_other.k_KHR) > i_float_tolerance ||
		abs(k_SHR - i_other.k_SHR) > i_float_tolerance ||
		abs(k_AHR - i_other.k_AHR) > i_float_tolerance ||
		v_niters != i_other.v_niters ||
		p_niters != i_other.p_niters ||
		d_niters != i_other.d_niters ||
		abs(k_LST - i_other.k_LST) > i_float_tolerance ||
		abs(k_AST - i_other.k_AST) > i_float_tolerance ||
		abs(k_VST - i_other.k_VST) > i_float_tolerance)
		return false;
	else
		return true;
}

//SoftBodyParameters& SoftBodyParameters::operator=(const SoftBodyParameters& other)
//{
//	k_ERP = other.k_ERP;
//	k_DP = other.k_DP;
//	k_PR = other.k_PR;
//	k_VC = other.k_VC;
//	k_DF = other.k_DF;
//	k_MT = other.k_MT;
//	k_CHR = other.k_CHR;
//	k_SHR = other.k_SHR;
//	k_AHR = other.k_AHR;
//	v_niters = other.v_niters;
//	p_niters = other.p_niters;
//	d_niters = other.d_niters;
//	k_LST = other.k_LST;
//	k_AST = other.k_AST;
//	k_VST = other.k_VST)
//}
