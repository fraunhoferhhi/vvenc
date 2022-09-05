# library
set( LIB_NAME apputils )

# create upper case name
string( TOUPPER ${LIB_NAME} LIB_NAME_UC )


# create file version.h
if(PROJECT_VERSION_RC)
  configure_file( ../vvenc/version-rc.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )
else()
  configure_file( ../vvenc/version.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )
endif()

# get include files
file( GLOB BASE_INC_FILES "*.h" )

# get public/extern include files
file( GLOB PUBLIC_INC_FILES  "../../../include/${LIB_NAME}/*.h" )


add_library( ${LIB_NAME} INTERFACE )

#target_compile_options( ${LIB_NAME} INTERFACE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Werror >
#                                             $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror >
#                                             $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4100 /wd4127 /wd4244 /wd4245 /wd4251 /wd4310 /wd4389 /wd4456 /wd4457 /wd4458 /wd4459 /wd4505 /wd4701 /wd4702 /wd4703 /wd4996 >)

target_include_directories( ${LIB_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> )


#  have the files showing in the IDE
add_custom_target( ${LIB_NAME}_h SOURCES ${INC_FILES} )

# example: place header files in different folders
source_group( "Header Files"  FILES ${INC_FILES} )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME}_h PROPERTIES FOLDER lib )