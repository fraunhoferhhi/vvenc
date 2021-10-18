# library
set( LIB_NAME apputils )

# create upper case name
string( TOUPPER ${LIB_NAME} LIB_NAME_UC )

# create file version.h
configure_file( version.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )

# get source files
file( GLOB BASE_SRC_FILES "*.cpp"  )

# get include files
file( GLOB BASE_INC_FILES "*.h" )

# get public/extern include files
file( GLOB PUBLIC_INC_FILES  "../../../include/${LIB_NAME}/*.h" )

# get all source files
set( SRC_FILES ${BASE_SRC_FILES} )

# get all include files
file( GLOB PRIVATE_INC_FILES ${BASE_INC_FILES} )

set( INC_FILES ${PRIVATE_INC_FILES} ${PUBLIC_INC_FILES}  )

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )

  # example: place header files in different folders
  source_group( "Natvis Files" FILES ${NATVIS_FILES} )
endif()

source_group( "Header Files"          FILES ${PUBLIC_INC_FILES} )
source_group( "Header Files\\private" FILES ${PRIVATE_INC_FILES} )


# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${LIB_NAME}.rc )
endif()

add_library( ${LIB_NAME} STATIC ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} ${RESOURCE_FILE} )

target_compile_definitions( ${LIB_NAME} PRIVATE ${LIB_NAME_UC}_SOURCE )
target_compile_definitions( ${LIB_NAME} PUBLIC $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${LIB_NAME},TYPE>,SHARED_LIBRARY>>:${LIB_NAME_UC}_DYN_LINK> )


target_compile_options( ${LIB_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Werror >
                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4100 /wd4127 /wd4244 /wd4245 /wd4251 /wd4310 /wd4389 /wd4456 /wd4457 /wd4458 /wd4459 /wd4505 /wd4701 /wd4702 /wd4703 /wd4996 >)


target_include_directories( ${LIB_NAME} PRIVATE           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
                                        SYSTEM INTERFACE  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> )
target_include_directories( ${LIB_NAME} PRIVATE . .. )

if( VVENC_ENABLE_THIRDPARTY_JSON )
  set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVVENC_ENABLE_THIRDPARTY_JSON" )
  set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVVENC_ENABLE_THIRDPARTY_JSON" )
endif()

if( TARGET INTEL_ITT)
  target_link_libraries( ${LIB_NAME} INTEL_ITT Threads::Threads vvenc )
else()
  target_link_libraries( ${LIB_NAME} Threads::Threads vvenc )
endif()

# set the folder where to place the projects
set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )
