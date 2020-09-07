
VVEncoderApp - VVC Encoder application with bitstream output
====================================================

The VVEncoderApp is a free application for encoding H.266/VVC bitstream from RAW YUV files.

What it does
------------

This tool encodes RAW YUV input with the H.266/VVC standard.


Output of this tool is decodable VVC bitstream.

Usage
-----

```
VVEncoderApp -i input.yuv [options] -o bitstreamfile
```

show help by running:

```
VVEncoderApp --help
VVEncoderApp --fullhelp
```

Application binaries
--------
The software supports builds for Windows, Linux and MacOSX.


Building
--------

The cmake build system scripts are provided to maintain cross platform experience and simplify generation of IDE-s projects like Visual Studio.

To use it

- install cmake from [cmake.org](http://cmake.org)
Build by Makefile usage:
- go into the project folder (where CMakeLists.txt is located)
- Build Binary by calling 'make release'
-
- Build/install everything for distribution:
- call 'make install' to build all suitable binaries, libraries and include directories; output is copied into the directory 'install' by default

- Manually build steps:
- create folder 'build' inside project root folder (where CMakeLists.txt is located)
- cd to this folder
- configure project using following command line cmake ..
- build project using your platform's default build system, say Visual Studio on Windows and make on UNIX


Restrictions
------------





