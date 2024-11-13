set( EXE_NAME vvenc_unit_test )

file( GLOB SRC_FILES CONFIGURE_DEPENDS "*.cpp" )
file( GLOB INC_FILES CONFIGURE_DEPENDS "*.h" )

add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} )
set_target_properties( ${EXE_NAME} PROPERTIES RELEASE_POSTFIX        "${CMAKE_RELEASE_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES DEBUG_POSTFIX          "${CMAKE_DEBUG_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}" )
set_target_properties( ${EXE_NAME} PROPERTIES MINSIZEREL_POSTFIX     "${CMAKE_MINSIZEREL_POSTFIX}" )
if( VVENC_LIBRARY_ONLY )
  set_target_properties( ${EXE_NAME} PROPERTIES EXCLUDE_FROM_ALL TRUE )
endif()

target_compile_options( ${EXE_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall>
                                            $<$<CXX_COMPILER_ID:GNU>:-Wall -fdiagnostics-show-option -Wno-ignored-attributes>

                                            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX /wd4244 /wd4251 /wd4996>)

target_include_directories( ${EXE_NAME} PRIVATE ${VVENC_LIB_DIR} )
target_link_libraries( ${EXE_NAME} Threads::Threads vvenc )

# Example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )

# Set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER test )
