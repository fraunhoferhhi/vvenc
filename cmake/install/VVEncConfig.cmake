# VVEncConfig.cmake - package configuration file

# get current directory
get_filename_component( SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH )

if( DEFINED VVEnc_USE_SHARED_LIBS )
  if( VVEnc_USE_SHARED_LIBS )
    include( ${SELF_DIR}/shared/VVEncTargets.cmake )
  else()
    include( ${SELF_DIR}/static/VVEncTargets.cmake )
  endif()
else()
  if( BUILD_SHARED_LIBS )
    include( ${SELF_DIR}/shared/VVEncTargets.cmake )
  else()
    include( ${SELF_DIR}/static/VVEncTargets.cmake )
  endif()  
endif()
