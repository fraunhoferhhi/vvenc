# enable testing with ctest
enable_testing()

set( TEST_YUV "${PROJECT_SOURCE_DIR}/test/data/RTn23_80x44p15_f15.yuv" )
set( TEST_CFG "${PROJECT_SOURCE_DIR}/test/data/RTn23.cfg" )
set( CFG_DIR  "${PROJECT_SOURCE_DIR}/cfg" )

# add test
add_test( NAME Test_vvencinterfacetest COMMAND vvencinterfacetest )

add_test( NAME Test_vvenclibtest-paramerter_range        COMMAND vvenclibtest 1 )
add_test( NAME Test_vvenclibtest-calling_order           COMMAND vvenclibtest 2 )
add_test( NAME Test_vvenclibtest-input_params            COMMAND vvenclibtest 3 )
add_test( NAME Test_vvenclibtest-sdk_default             COMMAND vvenclibtest 4 )
add_test( NAME Test_vvenclibtest-sdk_stringapi_interface COMMAND vvenclibtest 5 )
add_test( NAME Test_vvenclibtest-timestamps              COMMAND vvenclibtest 6 )

set( CLEANUP_TEST_FILES "" )

function( add_vvenc_test NAME TIMEOUT OUT_OUTPUT REQUIRES )
  set( command ${ARGN} )                                    # use remaining arguments after REQUIRES as commandline

  if( "OUTPUT" IN_LIST command )
    set( out_file "out_${NAME}.vvc" )                       # generate output file name
    list( TRANSFORM command REPLACE "OUTPUT" ${out_file} )  # substitute output file in command line
    set( ${OUT_OUTPUT} ${out_file} PARENT_SCOPE )           # return output file name in OUT_OUTPUT parameter

    set( CLEANUP_TEST_FILES "${CLEANUP_TEST_FILES};${out_file}" PARENT_SCOPE ) # collect list of output files to clean up afterwards
  endif()


  add_test( NAME Test_${NAME} COMMAND ${command} )
  set_tests_properties( Test_${NAME} PROPERTIES
                                     TIMEOUT ${TIMEOUT}
                                     FIXTURES_REQUIRED "${REQUIRES};cleanup"
                                     FIXTURES_SETUP "${out_file}"    # this test is the setup for the output-file
                      )
endfunction()

add_vvenc_test( vvencapp-tooltest       90 OUT_VVC   ""                       vvencapp --preset tooltest -s 80x44 -r 15 -i ${TEST_YUV} -f 8 -o OUTPUT )
add_vvenc_test( vvencFFapp-tooltest     60 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_tooltest.cfg -c ${CFG_DIR}/gop32.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 8 -b OUTPUT )
add_vvenc_test( compare_output-tooltest 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencFFapp-tooltest-Scalar     70 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_tooltest.cfg -c ${CFG_DIR}/gop32.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 8 --SIMD=SCALAR -b OUTPUT )
add_vvenc_test( compare_output-tooltest-Scalar 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencFFapp-tooltesttrans 20 OUT_VVC   "${OUTF_VVC}"            vvencFFapp -c ${CFG_DIR}/randomaccess_tooltest.cfg -c ${CFG_DIR}/gop32.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 8 --DebugBitstream=${OUTF_VVC} --DebugPOC=3 -b OUTPUT )
add_vvenc_test( output-tooltesttrans     30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-faster       30 OUT_VVC   ""                       vvencapp --preset faster -s 80x44 -r 15 -i ${TEST_YUV} -f 11 -o OUTPUT )
add_vvenc_test( vvencFFapp-faster     30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_faster.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 11 -b OUTPUT )
add_vvenc_test( compare_output-faster 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-fast       40 OUT_VVC   ""                       vvencapp --preset fast -s 80x44 -r 15 -i ${TEST_YUV} -f 8 -o OUTPUT )
add_vvenc_test( vvencFFapp-fast     30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_fast.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 8 -b OUTPUT )
add_vvenc_test( compare_output-fast 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencFFapp-transcoding 20 OUT_VVC   "${OUTF_VVC}"            vvencFFapp -c ${CFG_DIR}/randomaccess_fast.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 8 --DebugBitstream=${OUTF_VVC} --DebugPOC=3 -b OUTPUT )
add_vvenc_test( output-transcoding     30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-medium       30 OUT_VVC   ""                       vvencapp --preset medium -s 80x44 -r 15 -i ${TEST_YUV} -f 5 -o OUTPUT )
add_vvenc_test( vvencFFapp-medium     30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 5 -b OUTPUT )
add_vvenc_test( compare_output-medium 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-slow       90 OUT_VVC   ""                       vvencapp --preset slow -s 80x44 -r 15 -i ${TEST_YUV} -f 3 -o OUTPUT )
add_vvenc_test( vvencFFapp-slow     90 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_slow.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 3 -b OUTPUT )
add_vvenc_test( compare_output-slow 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-medium_noqpa_0thr       30 OUT_VVC   ""                       vvencapp --preset medium --qpa 0 --threads 0 -s 80x44 -r 15 -i ${TEST_YUV} -f 5 -o OUTPUT )
add_vvenc_test( vvencFFapp-medium_noqpa_0thr     30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} -qpa 0 --WppBitEqual=0 --Threads=0 -f 5 -b OUTPUT )
add_vvenc_test( compare_output-medium_noqpa_0thr 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencFFapp-lowdelay_medium_enc 30 OUTF_VVC  ""            vvencFFapp -c ${CFG_DIR}/experimental/lowdelay_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} --IntraPeriod=-1 -dph 1 -f 8 -b OUTPUT )
add_vvenc_test( vvencFFapp-lowdelay_medium_dec 30 NO_OUTPUT "${OUTF_VVC}" vvencFFapp --decode -b ${OUTF_VVC} )

add_vvenc_test( vvencapp-medium_rc2p       30 OUT_VVC   ""                       vvencapp --preset medium -s 80x44 -r 15 -i ${TEST_YUV} -f 5 --Bitrate=10000 --Passes=2 -o OUTPUT )
add_vvenc_test( vvencFFapp-medium_rc2p     30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 5 --TargetBitrate=10000 --Passes=2 -b OUTPUT )
add_vvenc_test( compare_output-medium_rc2p 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencFFapp-medium_rc2p_statsFile1_exp    30 OUTF_VVC  ""                       vvencFFapp -c ${CFG_DIR}/randomaccess_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 5 --TargetBitrate=10000 --Pass=1 --RCStatsFile=stats_exp.json -b OUTPUT )
add_vvenc_test( vvencFFapp-medium_rc2p_statsFile2_exp    30 OUTF_VVC  "${OUTF_VVC}"            vvencFFapp -c ${CFG_DIR}/randomaccess_medium.cfg -c ${TEST_CFG} -i ${TEST_YUV} -f 5 --TargetBitrate=10000 --Pass=2 --RCStatsFile=stats_exp.json -b OUTPUT )
add_vvenc_test( compare_output-medium_rc2p_statsFile_exp 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_vvenc_test( vvencapp-medium_rc2p_statsFile1_easy      30 OUT_VVC   ""                       vvencapp --preset medium -s 80x44 -r 15 -i ${TEST_YUV} -f 5 --Bitrate=10000 --Pass=1 --RCStatsFile=stats_easy.json -o OUTPUT )
add_vvenc_test( vvencapp-medium_rc2p_statsFile2_easy      30 OUT_VVC   "${OUT_VVC}"             vvencapp --preset medium -s 80x44 -r 15 -i ${TEST_YUV} -f 5 --Bitrate=10000 --Pass=2 --RCStatsFile=stats_easy.json -o OUTPUT )
add_vvenc_test( compare_output-medium_rc2p_statsFile_easy 30 NO_OUTPUT "${OUT_VVC};${OUTF_VVC}" ${CMAKE_COMMAND} -E compare_files ${OUT_VVC} ${OUTF_VVC} )

add_test( NAME Cleanup_remove_temp_files COMMAND ${CMAKE_COMMAND} -E remove -f ${CLEANUP_TEST_FILES} rec.yuv stats_exp.json stats_easy.json )
set_tests_properties( Cleanup_remove_temp_files PROPERTIES FIXTURES_CLEANUP cleanup )
