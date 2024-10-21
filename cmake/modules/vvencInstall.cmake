# vvencInstall

# set destination directories
set( RUNTIME_DEST ${CMAKE_INSTALL_BINDIR} )
set( LIBRARY_DEST ${CMAKE_INSTALL_LIBDIR} )
set( ARCHIVE_DEST ${CMAKE_INSTALL_LIBDIR} )

if( VVENC_INSTALL_FULLFEATURE_APP AND VVENC_LIBRARY_ONLY )
  message( FATAL_ERROR "VVENC_INSTALL_FULLFEATURE_APP conflicts with VVENC_LIBRARY_ONLY" )
endif()

set( VVENC_INST_TARGETS vvenc )

if( NOT VVENC_LIBRARY_ONLY )
  list( APPEND VVENC_INST_TARGETS vvencapp )

  if( VVENC_INSTALL_FULLFEATURE_APP )
    list( APPEND VVENC_INST_TARGETS vvencFFapp )
  endif()
endif()

# install targets
macro( install_targets config_ )
  string( TOLOWER ${config_} config_lc_ )
  install( TARGETS             ${VVENC_INST_TARGETS}
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
target_include_directories( vvenc  SYSTEM INTERFACE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> )

# install headers
install( FILES     ${CMAKE_BINARY_DIR}/vvenc/version.h  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/vvenc )
install( DIRECTORY include/vvenc                        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} )

# install targets
install_targets( Release )
install_targets( Debug )
install_targets( RelWithDebInfo )
install_targets( MinSizeRel )

# install pdb files
install_lib_pdb( vvenc )
if( NOT VVENC_LIBRARY_ONLY )
  install_exe_pdb( vvencapp )
  if( VVENC_INSTALL_FULLFEATURE_APP )
    install_exe_pdb( vvencFFapp )
  endif()
endif()

# configure version file
configure_file( cmake/install/vvencConfigVersion.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/vvencConfigVersion.cmake @ONLY )
# install cmake releated files
install( FILES cmake/install/vvencConfig.cmake                       DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/vvencConfigVersion.cmake  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )

# set config postfix
if( BUILD_SHARED_LIBS )
  set( CONFIG_POSTFIX shared )
else()
  set( CONFIG_POSTFIX static )
endif()

# create target cmake files
install( EXPORT vvencTargets-release        NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Release        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )
install( EXPORT vvencTargets-debug          NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS Debug          DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )
install( EXPORT vvencTargets-relwithdebinfo NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS RelWithDebInfo DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )
install( EXPORT vvencTargets-minsizerel     NAMESPACE vvenc:: FILE vvencTargets-${CONFIG_POSTFIX}.cmake CONFIGURATIONS MinSizeRel     DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vvenc )

function( resolve_target_interface_libs TGT OUT_VAR )
  get_target_property( interface_libs ${TGT} INTERFACE_LINK_LIBRARIES )

  foreach( lib ${interface_libs} )
    # extract actual target from generator expression
    if( ${lib} MATCHES  "<LINK_ONLY:\(.*\)>"  )
      set( lib ${CMAKE_MATCH_1} )
    endif()

    if( TARGET ${lib} )
      # if it is a target and not a -llibrary, we need to further resolve it
      resolve_target_interface_libs( ${lib} lib )
    endif()

    list( APPEND ret ${lib} )
  endforeach()

  set( ${OUT_VAR} ${ret} PARENT_SCOPE )
endfunction()

# create pkg-config file
set( VVENC_PKG_EXTRA_LIBS ${CMAKE_CXX_IMPLICIT_LINK_LIBRARIES} )

if( VVENC_PKG_EXTRA_LIBS )
  foreach( LIB ${VVENC_PKG_EXTRA_LIBS} )
    if((IS_ABSOLUTE ${LIB} AND EXISTS ${LIB}) OR (${LIB} MATCHES "^-"))
      list( APPEND EXTRALIBS ${LIB} )
    else()
       list( APPEND EXTRALIBS "-l${LIB}" )
    endif()
  endforeach()

  if( EXTRALIBS )
    set(VVENC_PKG_EXTRA_LIBS ${EXTRALIBS})
  endif()

  list( REMOVE_ITEM VVENC_PKG_EXTRA_LIBS "-lc" )
endif()

resolve_target_interface_libs( vvenc VVENC_PKG_INTERFACE_LIBS )
if( VVENC_PKG_INTERFACE_LIBS )
  list( APPEND VVENC_PKG_EXTRA_LIBS ${VVENC_PKG_INTERFACE_LIBS} )
endif()

list( JOIN VVENC_PKG_EXTRA_LIBS " " VVENC_PKG_EXTRA_LIBS  )
configure_file( pkgconfig/libvvenc.pc.in ${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/libvvenc.pc @ONLY )
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/pkgconfig/libvvenc.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig )

