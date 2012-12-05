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

#ifndef MPD_DEV_PERF_HISTOGRAM_H_
#define MPD_DEV_PERF_HISTOGRAM_H_

#include <deque>
#include <vector>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>

class PerformanceHistogram
{
public:
	PerformanceHistogram(int nvalues_, int nb_bars_ = 12);
	~PerformanceHistogram(void);

  void setColorSet(const std::vector<Eigen::Vector4f>& colors_);

	void render();

  void reset();

  void addMeasure(float measure_);

  void addMeasure(const std::vector<float>& measure_);

  bool isOverrun() const;

	int getHeight() const;

	int getWidth() const;

protected:

	int m_nbars;
  std::deque<std::vector<float> > m_measures; /**< Array of measures, from k to k - nbars (each value contains n values)*/
	std::deque<float>	m_measures_sum;					/**< Sum of values of each measure. */

  std::vector<Eigen::Vector4f> m_colors;    /**< measures colors (size nmeasures) */
  int m_nvalues;														/**< number of values per measure added. */

  int m_width;

  bool m_isOverrun;

  boost::mutex m_measures_mutex;              /**< Mutex for histogram values r/w. */
};

#endif // MPD_DEV_PERF_HISTOGRAM_H_
