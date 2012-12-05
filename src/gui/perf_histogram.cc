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

#include "gui/perf_histogram.h"
#include "SDL_OpenGL.h"
#include "gui/utils.h"

static const int HISTOGRAM_HEIGHT = 70;
static const int HISTOGRAM_BAR_WIDTH = 8;
static const int HISTOGRAM_BAR_GAP = 2;
static const float HISTOGRAM_OVERRUN_MAX = 1.5f; // 150% max overrun

PerformanceHistogram::PerformanceHistogram(int nvalues_, int nb_bars_) :
	m_nvalues(nvalues_),
	m_nbars(nb_bars_),
	m_isOverrun(false)
{
  m_width = m_nbars * HISTOGRAM_BAR_WIDTH + (m_nbars + 1) * HISTOGRAM_BAR_GAP;

  reset();

  //for (int i = 0 ; i < m_nbars ; ++i)
  //{
  //  //m_values[i] = i * 0.1f;
  //  std::vector<float> values;
  //  for (int j = 0 ; j < m_nvalues ; ++j)
  //  {
  //    values.push_back(i*0.1 + 0.05*j);
  //  }
  //  m_measures[i] = values;
  //}
  for (int i = 0 ; i < m_nvalues ; ++i)
    m_colors.push_back(int2Color(i+6));
}

PerformanceHistogram::~PerformanceHistogram(void) 
{}

int PerformanceHistogram::getHeight() const
{
	return HISTOGRAM_HEIGHT;
}

int PerformanceHistogram::getWidth() const
{
	return m_nbars * HISTOGRAM_BAR_WIDTH + (m_nbars+1) * HISTOGRAM_BAR_GAP;
}

void PerformanceHistogram::setColorSet(const std::vector<Eigen::Vector4f>& colors_)
{
  m_colors = colors_;
}

void PerformanceHistogram::render()
{
  static const int half_gap = HISTOGRAM_BAR_GAP / 2;
  static const int overrun_height = HISTOGRAM_HEIGHT / HISTOGRAM_OVERRUN_MAX;
  static const int overrun_margin = 2; // in px

  boost::mutex::scoped_lock vlock(m_measures_mutex);    // mutex for reading values

  m_isOverrun = false;
  //float tot_val = 0.f;
  //for (int j = 0 ; j < m_values.front().size() ; ++j)
  //  tot_val += m_values.front()[j];

  if (m_measures_sum.front() >= 1.f)
    m_isOverrun = true;

  glBegin(GL_QUADS);
  if (m_isOverrun)
  {
    glColor4ub(255,0,0,96);
    glVertex2i(-overrun_margin, -overrun_margin);
    glVertex2i(m_width+overrun_margin, -overrun_margin);
    glVertex2i(m_width+overrun_margin, HISTOGRAM_HEIGHT+overrun_margin);
    glVertex2i(-overrun_margin, HISTOGRAM_HEIGHT+overrun_margin);
  }

	glColor4ub(0,0,0,128);
  glVertex2i(0, 0);
	glVertex2i(m_width, 0);
	glVertex2i(m_width, HISTOGRAM_HEIGHT);
	glVertex2i(0, HISTOGRAM_HEIGHT);
	glEnd();

	glColor4ub(192, 192, 192, 255);
  glLineWidth(.5);
  glBegin(GL_LINES);
  glVertex2i(half_gap, overrun_height);
  glVertex2i(m_width - half_gap, overrun_height);
  glEnd();

  glBegin(GL_QUADS);
  for (int i = 0 ; i < m_measures.size() ; ++i)
  {
    const std::vector<float>& values = m_measures[i];
    int prev_y = 1;
    for (int j = 0 ; j < values.size() ; ++j)
    {
      //if (isOverruning)
      //  glColor4f(.8f, 0.f, 0.f, 1.f);
      //else
        glColor4fv(m_colors[j].data());
      const float val = values[j];
      const float val_norm = val / HISTOGRAM_OVERRUN_MAX;
      glVertex2i(i*HISTOGRAM_BAR_WIDTH + (i+1)*HISTOGRAM_BAR_GAP, prev_y);
      glVertex2i((i+1)*HISTOGRAM_BAR_WIDTH + (i+1)*HISTOGRAM_BAR_GAP, prev_y);
      prev_y = std::min(static_cast<int>((HISTOGRAM_HEIGHT-2)*val_norm) + prev_y, HISTOGRAM_HEIGHT-2);
      glVertex2i((i+1)*HISTOGRAM_BAR_WIDTH + (i+1)*HISTOGRAM_BAR_GAP, /*(HISTOGRAM_HEIGHT-2)*val_norm +*/ prev_y);
      glVertex2i(i*HISTOGRAM_BAR_WIDTH + (i+1)*HISTOGRAM_BAR_GAP, /*(HISTOGRAM_HEIGHT-2)*val_norm +*/ prev_y);
      prev_y = std::min(prev_y + 1, HISTOGRAM_HEIGHT-2);
    }
  }
  glEnd();

}

void PerformanceHistogram::addMeasure(float measure_)
{
  boost::mutex::scoped_lock vlock(m_measures_mutex); // mutex for writing measures

  m_measures.push_front(std::vector<float>(1, measure_)); 
  m_measures.pop_back();
	m_measures_sum.push_front(measure_);
  m_measures_sum.pop_back();
}

void PerformanceHistogram::addMeasure(const std::vector<float>& measure_)
{
  boost::mutex::scoped_lock vlock(m_measures_mutex); // mutex for writing measures

  m_measures.push_front(measure_); 
  m_measures.pop_back();

	float sum = 0.f;
	for (size_t i = 0 ; i < measure_.size() ; ++i)
		sum += measure_[i];

	m_measures_sum.push_front(sum);
  m_measures_sum.pop_back();
}

void PerformanceHistogram::reset()
{
  boost::mutex::scoped_lock vlock(m_measures_mutex); // mutex for writing values

  std::vector<float> empty_val(m_nvalues, 0.f);
  m_measures.assign(m_nbars, empty_val);
	m_measures_sum.assign(m_nbars, 0.f);
}

bool PerformanceHistogram::isOverrun() const
{
  return m_isOverrun;
}
