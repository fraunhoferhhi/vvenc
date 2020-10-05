# library
set( LIB_NAME vvenc )

# create upper case name
string( TOUPPER ${LIB_NAME} LIB_NAME_UC )

# create file version.h
configure_file( version.h.in "${CMAKE_BINARY_DIR}/${LIB_NAME}/version.h" )

# get source files
file( GLOB BASE_SRC_FILES "*.cpp"  "../CommonLib/*.cpp"  "../Utilities/*.cpp" "../DecoderLib/*.cpp" "../EncoderLib/*.cpp" )

# get include files
file( GLOB BASE_INC_FILES "*.h" "../CommonLib/*.h"  "../Utilities/*.h" "../DecoderLib/*.h" "../EncoderLib/*.h" )

# get x86 source files
file( GLOB X86_SRC_FILES "../CommonLib/x86/*.cpp" )

# get x86 include files
file( GLOB X86_INC_FILES "../CommonLib/x86/*.h" )

# get avx source files
file( GLOB AVX_SRC_FILES "../CommonLib/x86/avx/*.cpp" )

# get avx2 source files
file( GLOB AVX2_SRC_FILES "../CommonLib/x86/avx2/*.cpp" )

# get sse4.1 source files
file( GLOB SSE41_SRC_FILES "../CommonLib/x86/sse41/*.cpp" )

# get sse4.2 source files
file( GLOB SSE42_SRC_FILES "../CommonLib/x86/sse42/*.cpp" )

# get public/extern include files
file( GLOB PUBLIC_INC_FILES  "../../../include/${LIB_NAME}/*.h" )

# get all source files
set( SRC_FILES ${BASE_SRC_FILES} ${X86_SRC_FILES} ${SSE41_SRC_FILES} ${SSE42_SRC_FILES} ${AVX_SRC_FILES} ${AVX2_SRC_FILES} )

# get all include files
file( GLOB PRIVATE_INC_FILES ${BASE_INC_FILES} ${X86_INC_FILES}  )

set( INC_FILES ${PRIVATE_INC_FILES} ${PUBLIC_INC_FILES}  )

# NATVIS files for Visual Studio
if( MSVC )
  file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )

  # example: place header files in different folders
  source_group( "Natvis Files" FILES ${NATVIS_FILES} )
endif()

source_group( "Header Files"          FILES ${PUBLIC_INC_FILES} )
source_group( "Header Files\\private" FILES ${PRIVATE_INC_FILES} )

# set needed compile definitions
set_property( SOURCE ${SSE41_SRC_FILES} APPEND PROPERTY COMPILE_DEFINITIONS USE_SSE41 )
set_property( SOURCE ${SSE42_SRC_FILES} APPEND PROPERTY COMPILE_DEFINITIONS USE_SSE42 )
set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_DEFINITIONS USE_AVX )
set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_DEFINITIONS USE_AVX2 )
# set needed compile flags
if( MSVC )
  set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_FLAGS "/arch:AVX" )
  set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_FLAGS "/arch:AVX2" )
elseif( UNIX OR MINGW )
  set_property( SOURCE ${SSE41_SRC_FILES} APPEND PROPERTY COMPILE_FLAGS "-msse4.1" )
  set_property( SOURCE ${SSE42_SRC_FILES} APPEND PROPERTY COMPILE_FLAGS "-msse4.2" )
  set_property( SOURCE ${AVX_SRC_FILES}   APPEND PROPERTY COMPILE_FLAGS "-mavx" )
  set_property( SOURCE ${AVX2_SRC_FILES}  APPEND PROPERTY COMPILE_FLAGS "-mavx2" )
endif()

add_library( ${LIB_NAME} ${SRC_FILES} ${INC_FILES} ${NATVIS_FILES} )

target_compile_definitions( ${LIB_NAME} PRIVATE ${LIB_NAME_UC}_SOURCE )
target_compile_definitions( ${LIB_NAME} PUBLIC $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${LIB_NAME},TYPE>,SHARED_LIBRARY>>:${LIB_NAME_UC}_DYN_LINK> )


target_compile_options( ${LIB_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Werror -Wno-deprecated-register -Wno-unused-const-variable -Wno-unknown-attributes>
                                            $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-unused-function  -Wno-unused-variable  -Wno-sign-compare  -fdiagnostics-show-option>
                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4100 /wd4127 /wd4244 /wd4245 /wd4251 /wd4310 /wd4389 /wd4456 /wd4457 /wd4458 /wd4459 /wd4505 /wd4701 /wd4702 /wd4703 /wd4996 >)
                                            

target_include_directories( ${LIB_NAME} PRIVATE           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>                                       
                                        SYSTEM INTERFACE  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../include> $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}> )
target_include_directories( ${LIB_NAME} PRIVATE . .. ../DecoderLib ../EncoderLib ../CommonLib ../CommonLib/x86 )

target_link_libraries( ${LIB_NAME} Threads::Threads )

# set the folder where to place the projects
set_target_properties( ${LIB_NAME} PROPERTIES FOLDER lib )
