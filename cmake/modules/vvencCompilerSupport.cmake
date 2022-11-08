include( CheckCCompilerFlag )
include( CheckCSourceCompiles )

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

  set_if_compiler_supports_flag( FLAG_msse2 -msse2 )
  set_if_compiler_supports_flag( FLAG_mavx  -mavx  )

  _check_intrinsic( _mm_storeu_si16     "${FLAG_msse2}" "int16_t a = 0; _mm_storeu_si16( &a, _mm_setzero_si128() );" )
  _check_intrinsic( _mm_storeu_si32     "${FLAG_msse2}" "int32_t a = 0; _mm_storeu_si32( &a, _mm_setzero_si128() );" )
  _check_intrinsic( _mm_storeu_si64     "${FLAG_msse2}" "int64_t a = 0; _mm_storeu_si64( &a, _mm_setzero_si128() );" )
  _check_intrinsic( _mm_loadu_si32      "${FLAG_msse2}" "int32_t a = 0; __m128i x = _mm_loadu_si32( &a );"           )
  _check_intrinsic( _mm_loadu_si64      "${FLAG_msse2}" "int64_t a = 0; __m128i x = _mm_loadu_si64( &a );"           )
  _check_intrinsic( _mm256_zeroupper    "${FLAG_mavx}"  "_mm256_zeroupper();"                                        )
  _check_intrinsic( _mm256_loadu2_m128i "${FLAG_mavx}"  "int a[4] = { 0, };                                          \
                                                        __m256i x = _mm256_loadu2_m128i( ( const __m128i* ) a,       \
                                                                                         ( const __m128i* ) a );"    )
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

    #include <simde/x86/avx.h>
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
