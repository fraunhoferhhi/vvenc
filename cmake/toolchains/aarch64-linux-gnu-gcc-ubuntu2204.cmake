# name of the target operating system
set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_PROCESSOR aarch64 )

set( GNU_MACHINE "aarch64-linux-gnu" )

# which compilers to use for C and C++
set( CMAKE_C_COMPILER ${GNU_MACHINE}-gcc )
set( CMAKE_CXX_COMPILER ${GNU_MACHINE}-g++ )

# here is the target environment located
if( NOT DEFINED ARM_LINUX_SYSROOT AND DEFINED GNU_MACHINE )
  set( ARM_LINUX_SYSROOT /usr/${GNU_MACHINE}${FLOAT_ABI_SUFFIX} )
endif()

list( APPEND CMAKE_FIND_ROOT_PATH ${ARM_LINUX_SYSROOT} )

# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search 
# programs in the host environment
set( CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER )

# Ubuntu/amd64 + foreign architecture arm64
set( CMAKE_LIBRARY_PATH /usr/lib/${GNU_MACHINE}-linux-gnu )
set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH )
#set( CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY )

set( CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY )

set( USE_OPENCV_TOOLCHAIN_FLAGS ON )
if( USE_OPENCV_TOOLCHAIN_FLAGS )
  # ---
  # Snatched from OpenCV 3.4.2-1 
  # ---
  
  #set(CMAKE_CXX_FLAGS           "${CMAKE_CXX_FLAGS} -fdata-sections -Wa,--noexecstack -fsigned-char -Wno-psabi")
  #set(CMAKE_C_FLAGS             "${CMAKE_C_FLAGS} -fdata-sections -Wa,--noexecstack -fsigned-char -Wno-psabi")
  set( CMAKE_CXX_FLAGS_INIT      "-fdata-sections -Wa,--noexecstack -fsigned-char" )
  set( CMAKE_C_FLAGS_INIT        "-fdata-sections -Wa,--noexecstack -fsigned-char" )

  set(ARM_LINKER_FLAGS "-Wl,--no-undefined -Wl,--gc-sections -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")

  #set(CMAKE_SHARED_LINKER_FLAGS "${ARM_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS}")
  #set(CMAKE_MODULE_LINKER_FLAGS "${ARM_LINKER_FLAGS} ${CMAKE_MODULE_LINKER_FLAGS}")
  #set(CMAKE_EXE_LINKER_FLAGS    "${ARM_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS}")

  set( CMAKE_SHARED_LINKER_FLAGS_INIT "${ARM_LINKER_FLAGS}" )
  set( CMAKE_MODULE_LINKER_FLAGS_INIT "${ARM_LINKER_FLAGS}" )
  set( CMAKE_EXE_LINKER_FLAGS_INIT    "${ARM_LINKER_FLAGS}" )

endif()
