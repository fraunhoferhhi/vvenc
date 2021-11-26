# name of the target operating system
set( CMAKE_SYSTEM_NAME Windows )
set( CMAKE_SYSTEM_PROCESSOR x86 )

# which compilers to use for C and C++
set( CMAKE_C_COMPILER i686-w64-mingw32-gcc-posix )
set( CMAKE_CXX_COMPILER i686-w64-mingw32-g++-posix )
set( CMAKE_RC_COMPILER i686-w64-mingw32-windres )

# here is the target environment located
#SET(CMAKE_FIND_ROOT_PATH  /usr/i586-mingw32msvc /home/alex/mingw-install )
#
# /usr/share/mingw-w64/include
# /usr/x86_64-w64-mingw32/lib
# /usr/x86_64-w64-mingw32/include
# /usr/lib/gcc/x86_64-w64-mingw32/5.3-posix/include
# /usr/lib/gcc/x86_64-w64-mingw32/5.3-posix/libstdc++.a
set( CMAKE_FIND_ROOT_PATH /usr/share/mingw-w64 /usr/i686-w64-mingw32 /usr/lib/gcc/i686-w64-mingw32/5.3-posix )

# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search 
# programs in the host environment
set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER )
set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )
set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )

set( bb_MINGW_RUNTIME_FILES 
     /usr/lib/gcc/i686-w64-mingw32/5.3-posix/libstdc++-6.dll
     /usr/lib/gcc/i686-w64-mingw32/5.3-posix/libgcc_s_sjlj-1.dll
     /usr/i686-w64-mingw32/lib/libwinpthread-1.dll )
 