# executable
set( EXE_NAME vvencapp )

# get source files
file( GLOB SRC_FILES "*.cpp" )

# get include files
file( GLOB INC_FILES "*.h" )

# set resource file for MSVC compilers
if( MSVC )
  set( RESOURCE_FILE ${EXE_NAME}.rc )
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

target_compile_options( ${EXE_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:-Wall -Wno-unused-value>)

target_link_libraries( ${EXE_NAME} Threads::Threads vvenc )

# example: place header files in different folders
source_group( "Header Files"   FILES ${INC_FILES} )
source_group( "Resource Files" FILES ${RESOURCE_FILE} )


# set the folder where to place the projects
set_target_properties( ${EXE_NAME}  PROPERTIES FOLDER app )
