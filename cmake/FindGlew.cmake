include(FindPackageHandleStandardArgs)

find_path(Glew_LIBRARY_DIRS ${CMAKE_STATIC_LIBRARY_PREFIX}glew32${CMAKE_STATIC_LIBRARY_SUFFIX})

find_path(Glew_INCLUDE_DIRS glew.h PATH_SUFFIXES "GL"
    DOC "Location of CCD header files")

if(Glew_LIBRARY_DIRS AND Glew_INCLUDE_DIRS)
  set(Glew_LIBRARIES "glew32${CMAKE_STATIC_LIBRARY_SUFFIX}")
  message("Glew_LIBRARIES=${Glew_LIBRARIES}")
  set(Glew_FOUND 1)
else()
  message("Could not find Glew library")
  set(Glew_FOUND 0)
endif()

#find_package_handle_standard_args(Glew DEFAULT_MSG Glew_LIBRARIES Glew_INCLUDE_DIRS)
mark_as_advanced(Glew_INCLUDE_DIRS Glew_LIBRARY_DIRS Glew_LIBRARIES)
