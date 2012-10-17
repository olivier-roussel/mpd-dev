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

#include "gui/file_utils.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

std::vector<boost::filesystem::path> listDirectory(const boost::filesystem::path& dir, const std::string& ext, bool recurse)
{
  std::vector<boost::filesystem::path> res;
  const std::string upper_ext = boost::to_upper_copy(ext);
  if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir))
  {
    boost::filesystem::directory_iterator end_iter;
    for (boost::filesystem::directory_iterator dir_iter(dir) ; dir_iter != end_iter ; ++dir_iter)
    {
      const boost::filesystem::path& child = *dir_iter;
      if (boost::filesystem::is_regular_file(dir_iter->status()))
      {
        const std::string cur_ext = boost::to_upper_copy(child.extension().string());
        if (upper_ext == cur_ext)
          res.push_back(child);
      }else if(recurse && boost::filesystem::is_directory(child)) // recurse
      {
        const std::vector<boost::filesystem::path> rec_res = listDirectory(child, ext, true);
        res.insert(res.end(), rec_res.begin(), rec_res.end());
      }
    }
  }
  return res;
}

