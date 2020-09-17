# executable
set( EXE_NAME vvencFFapp )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${EXE_NAME}.rc )
    file( GLOB NATVIS_FILES "../../VisualStudio/*.natvis" )
  # extend the stack size on windows to 2MB
  set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} /STACK:0x200000" )
endif()

if( VVENC_ENABLE_APP_SHARED_POSTFIX AND BUILD_SHARED_LIBS )
  set( POSTFIX_APPEND "-s"  )
endif()

# add executable
add_executable( ${EXE_NAME} ${SRC_FILES} ${INC_FILES} ${RESOURCE_FILE} )
set_target_properties( ${EXE_NAME} PROPERTIES RELEASE_POSTFIX        "${CMAKE_RELEASE_POSTFIX}${POSTFIX_APPEND}" )
set_target_properties( ${EXE_NAME} PROPERTIES DEBUG_POSTFIX          "${CMAKE_DEBUG_POSTFIX}${POSTFIX_APPEND}" )
set_target_properties( ${EXE_NAME} PROPERTIES RELWITHDEBINFO_POSTFIX "${CMAKE_RELWITHDEBINFO_POSTFIX}${POSTFIX_APPEND}" )
set_target_properties( ${EXE_NAME} PROPERTIES MINSIZEREL_POSTFIX     "${CMAKE_MINSIZEREL_POSTFIX}${POSTFIX_APPEND}" )

target_link_libraries( ${EXE_NAME} Threads::Threads vvenc )

# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )
source_group( "Resource Files" FILES ${RESOURCE_FILE} )
source_group( "Natvis Files"   FILES ${NATVIS_FILES} )

# set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER app )
