
# DETECT_PLATFORM
# -------------
#
# Detect current plaform and set following
# variables :
#   PLATFORM_OS 
#   PLATFORM_ARCH 
#   PLATFORM_COMPILER 
#   PLATFORM 

MACRO(DETECT_PLATFORM)

  if(MSVC)
    set(PLATFORM_OS "win")
    set(PLATFORM_ARCH "x86")
    if(CMAKE_CL_64)
      set(PLATFORM_ARCH "x86_64")
    endif()
    
    if(MSVC_VERSION EQUAL 1400) # msvc8
      set(PLATFORM "msvc8")
      set(PLATFORM_COMPILER "vc80")
    endif()
    
    if(MSVC_VERSION EQUAL 1500) # msvc9
      set(PLATFORM "msvc9")
      set(PLATFORM_COMPILER "vc90")
    endif()
    
    if(MSVC_VERSION EQUAL 1600) # msvc10 - 2010
      set(PLATFORM "msvc10")
      set(PLATFORM_COMPILER "vc100")
    endif()
    
  endif(MSVC)

  if(UNIX)

    set(PLATFORM_OS "linux")
    set(PLATFORM_ARCH "x86")
    if(CMAKE_CL_64)
      set(PLATFORM_ARCH "x86_64")
    endif()
    set(PLATFORM_COMPILER "gcc4")
    
    # TODO : probably more stuff _ to be tested
    
  endif(UNIX)

  if ("${PLATFORM_OS}" STREQUAL "")
    message("Could not determine current OS")
  endif()
  if ("${PLATFORM_ARCH}" STREQUAL "")
    message("Could not determine current architecture")
  endif()
  if ("${PLATFORM_COMPILER}" STREQUAL "")
    message("Could not determine current compiler")
  endif()

ENDMACRO(DETECT_PLATFORM)
