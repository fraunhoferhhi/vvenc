# VVEncInstall

if( BUILD_SHARED_LIBS )
  set( CONFIG_POSTFIX shared )
else()
  set( CONFIG_POSTFIX static )
endif()

# set destination directories
set( RUNTIME_DEST bin/$<LOWER_CASE:$<CONFIG>>-${CONFIG_POSTFIX} )
set( LIBRARY_DEST lib/$<LOWER_CASE:$<CONFIG>>-${CONFIG_POSTFIX} )
set( ARCHIVE_DEST lib/$<LOWER_CASE:$<CONFIG>>-${CONFIG_POSTFIX} )

# install targets
macro( install_targets config_ )
  string( TOLOWER ${config_} config_lc_ )
  install( TARGETS             vvenc vvencapp vvencFFapp
           EXPORT              VVEncTargets-${config_lc_} 
           CONFIGURATIONS      ${config_}
           RUNTIME DESTINATION ${RUNTIME_DEST}
           LIBRARY DESTINATION ${LIBRARY_DEST}
           ARCHIVE DESTINATION ${ARCHIVE_DEST} )
endmacro( install_targets )

# install pdb file for static and shared libraries
macro( install_lib_pdb lib_ )
  if( MSVC )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,SHARED_LIBRARY>>:$<TARGET_PDB_FILE:${lib_}>>                                         
             CONFIGURATIONS Debug DESTINATION ${RUNTIME_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,SHARED_LIBRARY>>:$<TARGET_PDB_FILE:${lib_}>>                                         
             CONFIGURATIONS RelWithDebInfo DESTINATION ${RUNTIME_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/$<TARGET_PROPERTY:${lib_},NAME>.pdb> 
             CONFIGURATIONS Debug DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/$<TARGET_PROPERTY:${lib_},NAME>.pdb> 
             CONFIGURATIONS RelWithDebInfo DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    #install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/${lib_}.pdb> 
    #         CONFIGURATIONS Debug DESTINATION ${ARCHIVE_DEST} OPTIONAL )
    #install( FILES $<$<AND:$<PLATFORM_ID:Windows>,$<STREQUAL:$<TARGET_PROPERTY:${lib_},TYPE>,STATIC_LIBRARY>>:$<TARGET_FILE_DIR:${lib_}>/${lib_}.pdb> 
    #         CONFIGURATIONS RelWithDebInfo DESTINATION ${ARCHIVE_DEST} OPTIONAL )
  endif()
endmacro( install_lib_pdb )

# install pdb file for executables
macro( install_exe_pdb exe_ )
  if( MSVC )
    install( FILES $<$<PLATFORM_ID:Windows>:$<TARGET_PDB_FILE:${exe_}>>  DESTINATION ${RUNTIME_DEST} CONFIGURATIONS Debug          OPTIONAL )
    install( FILES $<$<PLATFORM_ID:Windows>:$<TARGET_PDB_FILE:${exe_}>>  DESTINATION ${RUNTIME_DEST} CONFIGURATIONS RelWithDebInfo OPTIONAL )
  endif()
endmacro( install_exe_pdb )

# set interface include directories
target_include_directories( vvenc  SYSTEM INTERFACE $<INSTALL_INTERFACE:include> )

# install headers
install( DIRECTORY include/vvenc  DESTINATION include )

# install targets
install_targets( Release )
install_targets( Debug )
install_targets( RelWithDebInfo )

# install pdb files
install_lib_pdb( vvenc )
install_exe_pdb( vvencapp )
install_exe_pdb( vvencFFapp )

# configure version file
configure_file( cmake/install/VVEncConfigVersion.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/VVEncConfigVersion.cmake @ONLY )
# install cmake releated files
install( FILES cmake/install/VVEncConfig.cmake                       DESTINATION lib/cmake/VVEnc )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/VVEncConfigVersion.cmake  DESTINATION lib/cmake/VVEnc )

# create target cmake files
install( EXPORT VVEncTargets-release        NAMESPACE vvenc:: FILE VVEncTargets.cmake CONFIGURATIONS Release        DESTINATION lib/cmake/VVEnc/${CONFIG_POSTFIX} )
install( EXPORT VVEncTargets-debug          NAMESPACE vvenc:: FILE VVEncTargets.cmake CONFIGURATIONS Debug          DESTINATION lib/cmake/VVEnc/${CONFIG_POSTFIX} )
install( EXPORT VVEncTargets-relwithdebinfo NAMESPACE vvenc:: FILE VVEncTargets.cmake CONFIGURATIONS RelWithDebInfo DESTINATION lib/cmake/VVEnc/${CONFIG_POSTFIX} )
