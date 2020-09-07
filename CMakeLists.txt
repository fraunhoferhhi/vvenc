# Top level CMakeLists.txt
#
# minimum required cmake version
cmake_minimum_required( VERSION 3.12.0 FATAL_ERROR )

# set cmake policy
if( NOT CMAKE_VERSION VERSION_LESS 3.12.0 )
  # Use latest policy
  cmake_policy( VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} )
endif()

# project name
project( vvenc )


# enable install target
set( VVENC_ENABLE_INSTALL ON CACHE BOOL "Enable or disable install target" )


# set default CMAKE_BUILD_TYPE to Release if not set
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE )
endif()

#if( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR
#    CMAKE_CXX_COMPILER_ID STREQUAL "Clang" )
#  set( USE_ADDRESS_SANITIZER OFF CACHE BOOL "Compiles and links with -fsanitize=address" )
#endif()

#set( ENABLE_LINK_TIME_OPT ON       CACHE BOOL   "Enable link time optimization for release and profile builds" )
#set( OPT_TARGET_ARCH      "native" CACHE STRING "Create code and optimize for this architecture (default native)" )

#set( SET_ENABLE_TRACING OFF CACHE BOOL "Set ENABLE_TRACING as a compiler flag" )
#set( ENABLE_TRACING OFF CACHE BOOL "If SET_ENABLE_TRACING is on, it will be set to this value" )

# set c++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# compile everything position independent (even static libraries)
set( CMAKE_POSITION_INDEPENDENT_CODE TRUE )

# set verbose compile options
#set( CMAKE_VERBOSE_MAKEFILE ON )

# use folders in IDEs for projects (e.g. lib sample app test)
set_property( GLOBAL PROPERTY USE_FOLDERS ON )

set( CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" )
message( STATUS "CMAKE_MODULE_PATH: updating module path to: ${CMAKE_MODULE_PATH}" )

# Enable multithreading
find_package( Threads REQUIRED )

# enable sse4.1 build for all source files for gcc and clang
if( UNIX OR MINGW )
  add_compile_options( "-msse4.1" )
endif()

# set _WIN32_WINNT
if( WIN32 )
  # set _WIN32_WINT version global
  add_definitions( -D_WIN32_WINNT=0x0600 )
endif()

# enable parallel build for Visual Studio
if( MSVC )
  # add compile options
  add_compile_options( /wd4310)
  add_compile_options( /wd4996)
  add_compile_options( "/MP" )
  add_compile_options( "/EHsc" )
endif()

# set address sanitizer compiler arguments
#if( USE_ADDRESS_SANITIZER )
#  # add compile options
#  add_compile_options( "-fsanitize=address" )
#  #add_compile_options( "-fno-omit-frame-pointer" )
#  #add_compile_options( "-fsanitize=leak" )
#  add_link_options( "-fsanitize=address" )
#  #add_link_options( "-fno-omit-frame-pointer" )
#  #add_link_options( "-fsanitize=leak" )
#endif()

# use ccache
find_program( CCACHE_FOUND ccache )
if( CCACHE_FOUND )
  message( STATUS "ccache found. using it." )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache )
  set_property( GLOBAL PROPERTY RULE_LAUNCH_LINK ccache )
endif()


add_subdirectory( "source/Lib/vvenc" )
add_subdirectory( "source/App/vvencapp" )
add_subdirectory( "source/App/vvencFFapp" )


if( VVENC_ENABLE_INSTALL )
  set( CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Standard install prefix" FORCE )
  include( cmake/modules/VVEncLibInstall.cmake )
endif()


