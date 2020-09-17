# vvencInstall

# set destination directories
set( RUNTIME_DEST bin )
set( LIBRARY_DEST lib )
set( ARCHIVE_DEST lib )

# install targets
macro( install_targets config_ )
  string( TOLOWER ${config_} config_lc_ )
  install( TARGETS             vvenc vvencapp vvencFFapp
           EXPORT              vvencTargets-${config_lc_} 
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
configure_file( cmake/install/vvencConfigVersion.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/vvencConfigVersion.cmake @ONLY )
# install cmake releated files
install( FILES cmake/install/vvencConfig.cmake                       DESTINATION lib/cmake/vvenc )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/vvencConfigVersion.cmake  DESTINATION lib/cmake/vvenc )

# set config postfix
if( BUILD_SHARED_LIBS )
  set( CONFIG_POSTFIX shared )
else()
  set( CONFIG_POSTFIX static )
endif()

# create target cmake files
install( EXPORT vvencTargets-release        NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Release        DESTINATION lib/cmake/vvenc )
install( EXPORT vvencTargets-debug          NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Debug          DESTINATION lib/cmake/vvenc )
install( EXPORT vvencTargets-relwithdebinfo NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS RelWithDebInfo DESTINATION lib/cmake/vvenc )
