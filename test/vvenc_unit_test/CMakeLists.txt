# executable
set( EXE_NAME vvenc_unit_test )

# get source files
file( GLOB SRC_FILES CONFIGURE_DEPENDS "*.cpp" )
# get include files
file( GLOB INC_FILES CONFIGURE_DEPENDS "*.h" )

# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${EXE_NAME}.rc )
endif()

# add executable
add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} ${RESOURCE_FILE} )
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
target_include_directories( ${EXE_NAME} PRIVATE $<$<TARGET_EXISTS:nlohmann_json::nlohmann_json>:$<TARGET_PROPERTY:nlohmann_json::nlohmann_json,INTERFACE_INCLUDE_DIRECTORIES>> )
target_compile_options( ${EXE_NAME} PRIVATE $<$<TARGET_EXISTS:nlohmann_json::nlohmann_json>:$<TARGET_PROPERTY:nlohmann_json::nlohmann_json,INTERFACE_COMPILE_OPTIONS>> )
target_compile_definitions( ${EXE_NAME} PRIVATE $<$<TARGET_EXISTS:nlohmann_json::nlohmann_json>:$<TARGET_PROPERTY:nlohmann_json::nlohmann_json,INTERFACE_COMPILE_DEFINITIONS>> )
# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )
source_group( "Resource Files" FILES ${RESOURCE_FILE} )


# set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER test )
