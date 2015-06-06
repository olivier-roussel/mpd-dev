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


#ifndef MPD_DEV_COMMON_UTILS_H_
#define MPD_DEV_COMMON_UTILS_H_

#include <boost\math\special_functions\fpclassify.hpp>

#define STATIC_ASSERT( x ) typedef char __STATIC_ASSERT__[( x )?1:-1]

/**
* \brief Returns true if all 3 vector elements are finite numbers.
* \pre Vector type T must have operator[] defined to acces to its 3
* elements as T[0], T[1] and T[2]
*/
template<class T>
bool isValidVector3(const T& i_vec)
{
	return boost::math::isfinite(i_vec[0]) && 
		boost::math::isfinite(i_vec[1]) &&
		boost::math::isfinite(i_vec[2]);
}

#define DEFINE_SIMPLE_ENUM(EnumName, seq) \
struct EnumName {\
   enum type \
   { \
      BOOST_PP_SEQ_FOR_EACH_I(DEFINE_SIMPLE_ENUM_VAL, EnumName, seq)\
   }; \
   type v; \
   EnumName(type v) : v(v) {} \
   operator type() const {return v;} \
private: \
    template<typename T> \
    operator T () const;};\

#define DEFINE_SIMPLE_ENUM_VAL(r, data, i, record) \
    BOOST_PP_TUPLE_ELEM(2, 0, record) = BOOST_PP_TUPLE_ELEM(2, 1, record),

#endif // MPD_DEV_COMMON_UTILS_H_
