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

#ifndef FILE_UTILS_H_
#define FILE_UTILS_H_

#ifdef WIN32
#	include <io.h>
#else
#	include <dirent.h>
#endif

#include <vector>

struct FileList
{
	static const int MAX_FILES = 256;
	inline FileList() : size(0) {}
	inline ~FileList()
	{
		clear();
	}
	
	void clear()
	{
		for (int i = 0; i < size; ++i)
			delete [] files[i];
		size = 0;
	}
	
	void add(const char* path)
	{
		if (size >= MAX_FILES)
			return;
		int n = strlen(path);
		files[size] = new char[n+1];
		strcpy(files[size], path);
		size++;
	}
	
	static int cmp(const void* a, const void* b)
	{
		return strcmp(*(const char**)a, *(const char**)b);
	}
	
	void sort()
	{
		if (size > 1)
			qsort(files, size, sizeof(char*), cmp);
	}
	
	char* files[MAX_FILES];
	int size;
};

static void scanDirectory(const char* path, const std::vector<std::string>& exts, FileList& list)
{
	list.clear();
	
#ifdef WIN32
  for (int i = 0 ; i < exts.size() ; ++i)
  {
	  _finddata_t dir;
	  char pathWithExt[MAX_PATH];
	  long fh;
	  strcpy(pathWithExt, path);
	  strcat(pathWithExt, "/*");
	  strcat(pathWithExt, exts[i].c_str());
	  fh = _findfirst(pathWithExt, &dir);
	  if (fh == -1L)
		  return;
	  do
	  {
		  list.add(dir.name);
	  } while (_findnext(fh, &dir) == 0);
	  _findclose(fh);
  }
#else
  // todo ! multi extenstion management
  //dirent* current = 0;
	//DIR* dp = opendir(path);
	//if (!dp)
	//	return;
	//
	//while ((current = readdir(dp)) != 0)
	//{
	//	int len = strlen(current->d_name);
	//	if (len > 4 && strncmp(current->d_name+len-4, ext, 4) == 0)
	//	{
	//		list.add(current->d_name);
	//	}
	//}
	//closedir(dp);
#endif
	list.sort();

}

#endif // FILE_UTILS_H_

// cmake:sourcegroup=Gui
