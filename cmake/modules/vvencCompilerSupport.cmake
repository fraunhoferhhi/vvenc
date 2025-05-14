include( CheckCCompilerFlag )
include( CheckCSourceCompiles )
include( CheckCXXSourceCompiles )

function( set_if_compiler_supports_flag output_var flag )
  string( REGEX REPLACE "[-.]" "_" SUPPORTED_flag_var "SUPPORTED${flag}" )

  check_c_compiler_flag( -Werror=unused-command-line-argument SUPPORTED_Werror_unused_command_line_argument )
  if( SUPPORTED_Werror_unused_command_line_argument )
    set( FLAGS_CHECK_STR "${flag} -Werror=unused-command-line-argument" )
  else()
    set( FLAGS_CHECK_STR "${flag}" )
  endif()

  # if building WASM -msimd128 needs to be added to all checks for supported simd-compiler flags
  _emscripten_enable_wasm_simd128()

  check_c_compiler_flag( ${FLAGS_CHECK_STR} ${SUPPORTED_flag_var} )
  if( ${SUPPORTED_flag_var} )
    set( ${output_var} "${flag}" PARENT_SCOPE )
  endif()
endfunction()


function( check_missing_intrinsics )
  # if building WASM -msimd128 needs to be added to all checks for supported simd-compiler flags
  _emscripten_enable_wasm_simd128()

  if( NOT MSVC )
    set_if_compiler_supports_flag( FLAG_msse41 -msse4.1 )
    set_if_compiler_supports_flag( FLAG_mavx   -mavx  )
  else()
    set_if_compiler_supports_flag( FLAG_mavx   /arch:AVX2  )
  endif()

  # SSE2
  _check_intrinsic( _mm_storeu_si16      "${FLAG_msse41}" "int16_t a = 0; _mm_storeu_si16( &a, _mm_setzero_si128() );"                )
  _check_intrinsic( _mm_storeu_si32      "${FLAG_msse41}" "int32_t a = 0; _mm_storeu_si32( &a, _mm_setzero_si128() );"                )
  _check_intrinsic( _mm_storeu_si64      "${FLAG_msse41}" "int64_t a = 0; _mm_storeu_si64( &a, _mm_setzero_si128() );"                )
  _check_intrinsic( _mm_loadu_si32       "${FLAG_msse41}" "int32_t a = 0; __m128i x = _mm_loadu_si32( &a );"                          )
  _check_intrinsic( _mm_loadu_si64       "${FLAG_msse41}" "int64_t a = 0; __m128i x = _mm_loadu_si64( &a );"                          )
  _check_intrinsic( _mm_cvtsi128_si64    "${FLAG_msse41}" "int64_t a = 0; a = _mm_cvtsi128_si64( _mm_setzero_si128() );"              )
  _check_intrinsic( _mm_cvtsi64_si128    "${FLAG_msse41}" "int64_t a = 0; __m128i x = _mm_cvtsi64_si128( a );"                        )  
  _check_intrinsic( _mm_extract_epi64    "${FLAG_msse41}" "int64_t a = 0; a = _mm_extract_epi64( _mm_setzero_si128(), 0 );"           )

  # AVX
  _check_intrinsic( _mm256_zeroupper     "${FLAG_mavx}"   "_mm256_zeroupper();"                                                       )
  _check_intrinsic( _mm256_loadu2_m128i  "${FLAG_mavx}"   "int a[4] = { 0, };                                                         \
                                                          __m256i x = _mm256_loadu2_m128i( (const __m128i*)a, (const __m128i*)a );"   )
  _check_intrinsic( _mm256_set_m128i     "${FLAG_mavx}"   "__m256i x = _mm256_set_m128i( _mm_setzero_si128(), _mm_setzero_si128() );" )
endfunction()

function( _check_intrinsic symbol_name compiler_flags code )
  set( CMAKE_REQUIRED_FLAGS       "${CMAKE_REQUIRED_FLAGS} ${compiler_flags}")
  set( CMAKE_REQUIRED_INCLUDES    "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/" )

  check_c_source_compiles("
    #include <simde/simde-arch.h>
    #if defined(SIMDE_ARCH_AMD64) || defined(SIMDE_ARCH_X86) || defined(SIMDE_ARCH_WASM)
    # include <immintrin.h>     // need this first, otherwise we get linker-errors on windows
    #else
    # define SIMDE_ENABLE_NATIVE_ALIASES
    #endif

    #include <simde/x86/avx2.h>
    #include <stdint.h>

    int main()
    {
      ${code}
      return 0;
    }"
    HAVE_INTRIN${symbol_name} )

  if( NOT HAVE_INTRIN${symbol_name} )
    add_compile_definitions( MISSING_INTRIN${symbol_name} )
  endif()
endfunction()

function( _emscripten_enable_wasm_simd128 )
  if( ${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten" )
    # this is only used by try_compile(), check_c_compiler_flag(), and related functions,
    # but not by the actual build.
    set( CMAKE_REQUIRED_FLAGS -msimd128 PARENT_SCOPE )
  endif()
endfunction()

function( _set_if_compiler_supports_sve_flag output_var sve_flag )
  set_if_compiler_supports_flag( tmp_var "${sve_flag}" )
  if( NOT tmp_var )
    return()
  endif()

  set( OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS} )
  set( OLD_CMAKE_TRY_COMPILE_TARGET_TYPE ${CMAKE_TRY_COMPILE_TARGET_TYPE} )
  set( CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} ${sve_flag}" )
  set( CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY )

  # Check whether the compiler can compile SVE functions that require
  # backup/restore of SVE registers according to AAPCS.
  # https://github.com/llvm/llvm-project/issues/80009.
  set( SVE_COMPILATION_TEST "
#include <arm_sve.h>
void other();
svfloat32_t func(svfloat32_t a) {
other();
return a;
}
int main() { return 0; }" )

  check_c_source_compiles( "${SVE_COMPILATION_TEST}" SVE_COMPILATION_C_TEST_COMPILED )
  check_cxx_source_compiles( "${SVE_COMPILATION_TEST}" SVE_COMPILATION_CXX_TEST_COMPILED )

  # Check if arm_neon_sve_bridge.h is available.
  set( SVE_HEADER_TEST "
#ifndef __ARM_NEON_SVE_BRIDGE
#error 1
#endif
#include <arm_sve.h>
#include <arm_neon_sve_bridge.h>
int main() { return 0; }")
  check_c_source_compiles( "${SVE_HEADER_TEST}" SVE_HEADER_C_TEST_COMPILED )
  check_cxx_source_compiles( "${SVE_HEADER_TEST}" SVE_HEADER_CXX_TEST_COMPILED )

  set( CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS} )
  set( CMAKE_TRY_COMPILE_TARGET_TYPE ${OLD_CMAKE_TRY_COMPILE_TARGET_TYPE} )

  if( SVE_COMPILATION_C_TEST_COMPILED AND SVE_COMPILATION_CXX_TEST_COMPILED AND
      SVE_HEADER_C_TEST_COMPILED AND SVE_HEADER_CXX_TEST_COMPILED )
    set( ${output_var} "${tmp_var}" PARENT_SCOPE )
  endif()
endfunction()

# Check if the compiler supports the AArch64 SVE and SVE2 extensions, and set
# variables for flags used to enable them to avoid duplication.
function( set_if_compiler_supports_arm_extensions output_flag_sve output_flag_sve2 )
  if( NOT(( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm64" ) OR
          ( ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64" )))
    return()
  endif()
  if( UNIX OR MINGW )
    # SVE is an optional feature from Armv8.2-A.
    set( _flag_sve "-march=armv8.2-a+sve" )
    _set_if_compiler_supports_sve_flag( _sve_supported "${_flag_sve}" )
    set( ${output_flag_sve} "${_sve_supported}" PARENT_SCOPE )

    # SVE2 is an optional feature from Armv9.0-A.
    set( _flag_sve2 "-march=armv9-a+sve2" )
    _set_if_compiler_supports_sve_flag( _sve2_supported "${_flag_sve2}" )
    set( ${output_flag_sve2} "${_sve2_supported}" PARENT_SCOPE )
  endif()
endfunction()

function( check_problematic_compiler output_var compiler_id first_bad_version first_fixed_version )
  if( CMAKE_CXX_COMPILER_ID STREQUAL "${compiler_id}"
      AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL "${first_bad_version}"
      AND (
        NOT "${first_fixed_version}"
        OR CMAKE_CXX_COMPILER_VERSION VERSION_LESS "${first_fixed_version}" ) )

    set( ${output_var} TRUE PARENT_SCOPE )

    if( "${first_fixed_version}" )
      set( ${output_var}_VERSION_RANGE "(${first_bad_version}...${first_fixed_version}]" PARENT_SCOPE )
    else()
      set( ${output_var}_VERSION_RANGE "(${first_bad_version}...)"                       PARENT_SCOPE )
    endif()

  endif()
endfunction()


function( detect_target_architecture output_var )
  # try to detect the actual target architecture
  if( ${CMAKE_SYSTEM_NAME} STREQUAL "Emscripten" )
    message( DEBUG  "Emscripten" )
    # Emscripten doesn't set the CMAKE_SYSTEM_PROCESSOR
    list( PREPEND target_arch_list "WASM" )
  elseif( NOT CMAKE_SYSTEM_PROCESSOR STREQUAL CMAKE_HOST_SYSTEM_PROCESSOR )
    message( DEBUG  "sys != host ${CMAKE_SYSTEM_PROCESSOR} STREQUAL ${CMAKE_HOST_SYSTEM_PROCESSOR} " )
    # cross compiling: CMAKE_SYSTEM_PROCESSOR was set explicitly, so we use that as first guess
    _append_cpu_type_guess( target_arch_list CMAKE_SYSTEM_PROCESSOR )
  endif()

  # build list of architectures in order of probability
  _append_cpu_type_guess( target_arch_list CMAKE_VS_PLATFORM_NAME )
  _append_cpu_type_guess( target_arch_list CMAKE_OSX_ARCHITECTURES )
  _append_cpu_type_guess( target_arch_list CMAKE_CXX_COMPILER_ARCHITECTURE_ID ) # set by msvc, wen not using msbuild
  _append_cpu_type_guess( target_arch_list CMAKE_ANDROID_ARCH_ABI )
  _append_cpu_type_guess( target_arch_list CMAKE_C_LIBRARY_ARCHITECTURE )
  _append_cpu_type_guess( target_arch_list CMAKE_SYSTEM_PROCESSOR )
  _append_cpu_type_guess( target_arch_list CMAKE_HOST_SYSTEM_PROCESSOR )
  list( APPEND target_arch_list "UNKNOWN" ) # no architecture for which we have specific optimizations
  message( DEBUG "target_arch_list: ${target_arch_list}" )

  # get most probable architecture
  list( POP_FRONT target_arch_list detected_arch )
  message( STATUS "normalized target architecture: ${detected_arch}" )

  set( ${output_var} "${detected_arch}" PARENT_SCOPE )
endfunction()

function( _append_cpu_type_guess output_list input_var )
  message( DEBUG "cpu_type_guess: input ${input_var}=${${input_var}}" )
  set( input_str ${${input_var}} )
  set( ret ${${output_list}} )

  string( TOLOWER "${input_str}" input_lower )
  if( ${input_lower} MATCHES "x86\|i386\|x64\|win32\|amd64" )
    list( APPEND ret "X86" )
  elseif( ${input_lower} MATCHES "aarch64\|arm64")
    list( APPEND ret "AARCH64" )
  elseif( ${input_lower} MATCHES "arm")
    list( APPEND ret "ARM" )
  elseif( ${input_lower} MATCHES "loongarch64")
    list( APPEND ret "LOONGARCH64" )
  endif()

  set( ${output_list} ${ret} PARENT_SCOPE )
endfunction()
