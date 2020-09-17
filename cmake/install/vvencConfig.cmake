# vvencConfig.cmake - package configuration file

# get current directory
get_filename_component( SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

# detect state
if( DEFINED vvenc_USE_SHARED_LIBS )
  if( vvenc_USE_SHARED_LIBS )
    set( USE_SHARED TRUE )
  else()
    set( USE_SHARED FALSE )
  endif()
else()
  if( BUILD_SHARED_LIBS )
    set( USE_SHARED TRUE )
  else()
    set( USE_SHARED FALSE )
  endif()  
endif()

if( USE_SHARED )
  if( EXISTS ${SELF_DIR}/vvencTargets-shared.cmake )
    include( ${SELF_DIR}/vvencTargets-shared.cmake )
  else()  
    include( ${SELF_DIR}/vvencTargets-static.cmake )
  endif()
else()
  if( EXISTS ${SELF_DIR}/vvencTargets-static.cmake )
    include( ${SELF_DIR}/vvencTargets-static.cmake )
  else()  
    include( ${SELF_DIR}/vvencTargets-shared.cmake )
  endif()
endif()
