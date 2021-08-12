# Fraunhofer Versatile Video Encoder (VVenC)

Versatile Video Coding (VVC) is the most recent international video coding standard, developed by the Joint Video Experts Team (JVET) of the ITU-T Video Coding Experts Group (VCEG) and the ISO/IEC Moving Picture Experts Group (MPEG). VVC is the successor of the High Efficiency Video Coding (HEVC) standard and is published by ITU-T as H.266 and by ISO/IEC as MPEG-I Part 3 (ISO/IEC 23090-3). The new standard targets a 50% bit-rate reduction over HEVC at the same visual quality. In addition, VVC proves to be truly versatile by including tools for efficient coding of video content in emerging applications, e.g. high dynamic range (HDR), adaptive streaming, computer generated content as well as immersive applications like 360-degree video and augmented reality (AR).

The Fraunhofer Versatile Video Encoder (VVenC) is a fast and efficient "real-world" VVC encoder implementation with the following main features:
- Easy to use encoder implementation with five predefined quality/speed presets;
- Perceptual optimization to improve subjective video quality, based on the XPSNR visual model;
- Frame-level single-pass and two-pass rate control supporting variable bit-rate (VBR) encoding;
- Expert mode encoder interface available, allowing fine-grained control of the encoding process.


#  How to build VVenC?

The software uses CMake to create platform specific build files.
A working CMake installation is required for building the software.
Download CMake from http://www.cmake.org/ and install it. The following targets are supported: Windows (Visual Studio), Linux (gcc) and MacOS (clang).

## How to build for Windows?
In order to compile the software for Windows, Visual Studio 15 2017 or higher and cmake version 3.13 or higher are required. Install gnuwin32 that provides make for Windows. To build the software open a command prompt window, change into the project directory and use:

    make install-release

This will create the statically linked release version of the encoder applications in the install/bin/release-static/ subdirectory.

## How to build for Linux/MacOS?
In order to compile the software for Linux, gcc version 7.0 or higher and cmake version 3.13 or higher are required. For MacOS, Xcode and cmake version 3.13 or higher are required. To simplify the build process a Makefile with predefined targets is available. To build the VVenC encoder applications open a terminal, change into the project directory and use:

    make install-release

This will create the statically linked release version of the encoder applications in the install/bin/release-static/ subdirectory.


# How to use VVenC?

The encoder project includes two encoder executables, a standard encoder (vvencapp) and a full featured expert encoder (vvencFFapp).

## How to use the standard encoder?
The standard encoder (**vvencapp**) can be used in one of five predefined presets. Each preset represents a different tradeoff between encoder runtime and video quality. In the slowest preset, the encoder reaches the highest compression gain, whilst in the fastest preset the runtime is significantly decreased. A list of the main encoder command line parameters is shown in the following table.

| OPTION                 | DEFAULT                          | DESCRIPTION                                                                                          |
|------------------------|----------------------------------|------------------------------------------------------------------------------------------------------|
| --help,-h              | -                                | Show basic help                                                                                      |
| --input,-i <str>       | -                                | Raw yuv input file or '-' for reading from stdin                                                                                   |
| --size,-s <wxh>        | 1920x1080                        | Input file resolution (width x height)                                                               |
| --framerate,-r <int>   | 60                               | Temporal rate of input file. Required for VBR encoding and calculation of output bit-rate. <br> Also recommended with perceptual QP adaptation (see `--qpa` option below). |
| --format,-c <str>      | yuv420                           | Set input format to YUV 4:2:0 8bit (yuv420) or YUV 4:2:0 10bit (yuv420_10)                           |
| --output,-o <str>      | not set                          | Bit-stream output file                                                                               |
| --preset <str>         | medium                           | Preset for specific encoding setting (faster, fast, medium, slow, slower)                            |
| --qp,-q <int>          | 32                               | Quantization parameter (0..63)                                                                       |
| --bitrate,-b <int>     | 0                                | Bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits per second). <br> Rate control requires correct `--framerate` (see option above). |
| --passes,-p <int>      | -1                               | Number of rate control passes (1: single-pass rate control, 2: two-pass rate control)                |
| --qpa <int>            | 1                                | Perceptual QP adaptation (QPA) to improve subjective video quality (0: off, 1: on)                   |
| --refreshsec,-rs <int> | 1                                | Intra period/refresh in seconds                                                                      |
| --threads,-t <int>     | size >= 1280x720: <br> 8, else: 4  | Number of threads (1-N)                                                                              |
| --hdr <str>            | off                              | HDR mode (+ SEI messages) + BT.709 or BT.2020 color space (off, pq, pq_2020, hlg, hlg_2020)          |

**Example usage:** Given a YUV 4:2:0 input file with a bit-depth of 8bit and a resolution of 176x144 pixels, the following call will encode the input file with the medium speedup preset:

    vvencapp --preset medium -i BUS_176x144_75@15.yuv -s 176x144 -o str.266

## How to use the full featured expert mode encoder?
The expert mode encoder (**vvencFFapp**) is based on the [VVC test model (VTM)](https://vcgit.hhi.fraunhofer.de/jvet/VVCSoftware_VTM) reference software configuration scheme. Most of the parameters have been kept similar to VTM, but for some parameters, additional modes are available. Furthermore, not supported options have been removed. The following example configuration files for the expert mode encoder can be found in the cfg sub-directory:

| CONFIGURATION FILE                                                                                                              | DESCRIPTION                                                                                                             |
|---------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
| sequence.cfg                                                                                                                    | Sequence specific configuration parameters. Must be always adapted to the input sequence.                               |
| randomaccess_faster.cfg<br>randomaccess_fast.cfg<br>randomaccess_medium.cfg<br>randomaccess_slow.cfg<br>randomaccess_slower.cfg | Random access configuration for different presets. Each configuration file corresponds to one of the 5 preset modes.    |
| qpa.cfg                                                                                                                         | Perceptually optimized QPA configuration file.                                                                          |
| rc1p.cfg                                                                                                                        | Single pass rate control configuration, overriding default fix QP setup.                                                |
| rc2p.cfg                                                                                                                        | Two pass rate control configuration, overriding default fix QP setup.                                                   |

**Example usage:** In order to start your first experiments with the expert mode encoder, adapt the sequence.cfg configuration file to your input YUV source file and use the following command:

    vvencFFapp -c randomaccess_medium.cfg -c sequence.cfg
    
## How to map command line parameters of the standard encoder into the full featured expert mode encoder?
The export mode encoder (**vvencFFapp**) can be used in the same manner as the standard encoder (**vvencapp**) by using the adapted expert option names.
Be aware of that some options have different default values. The following table shows only expert options that differs from the standard encoder:

| OPTION  (standard)     | DEFAULT                          | OPTION (full feature)  | DEFAULT                          |
|------------------------|----------------------------------|------------------------|----------------------------------|
| --input,-i <str>       | -                                | --InputFile <str>      | -                                |
| --size,-s <wxh>        | 1920x1080                        | --Size,-s <wxh>        | 0x0                              |
|                        |                                  | --SourceWidth <int> --SourceHeight <int> | 0              |
| --format,-c <str>      | yuv420                           | --InputBitDepth <int>  | 8                                |
| --internal-bitdepth <int> | 10                            | --InternalBitDepth <int> | 10                             |
| --framerate,-r <int>   | 60                               | --FrameRate,-fr <int>  | 0                                |
| --tickspersec <int>    | 90000                            | --TicksPerSecond <int> | 90000                            |
| --frames <int>         | 0                                | --FramesToBeEncoded <int> | 0                             |
| --frameskip <int>      | 0                                | --FrameSkip <int>      | 0                                |
| --output,-o <str>      | not set                          | --BitstreamFile,-b <str> | not set                        |
| --qp,-q <int>          | 32                               | --QP <int>             | 32                               |
| --bitrate,-b <int>     | 0                                | --TargetBitrate <int>  | 0                                |
| --passes,-p <int>      | -1                                | --NumPasses <int>     | -1                               |
| --qpa <int>            | 1                                | --PerceptQPA,-qpa <int> | 0                               |
| --refreshtype,-rt <str> | cra                             | --DecodingRefreshType <str> | cra                         |
| --refreshsec,rs <int>  | 1                                | --RefreshSec <int>       | 1                              |
| --gopsize,-g <int>     | 32                               | --GOPSize <int>         | 32                              |
| --threads,-t <int>     | size >= 1280x720: <br> 8, else: 4 | --Threads,-t <int>    | 0                                |
| --hdr <str>            | off                              | --Hdr <str>            | off                              |
| --profile <str>        | auto                             | --Profile <str>        | auto                             |
| --level <str>          | auto                             | --Level <str>          | auto                             |
| --tier <str>           | auto                             | --Tier <str>           | auto                             |
| --accessunitdelimiter,-aud <int> | auto                   | --AccessUnitDelimiter,-aud <int> | auto                   |
| --vuiparameterspresent,-vui <int> | auto                  | --VuiParametersPresent,-vui <int> | auto                  |
| --hrdparameterspresent,-hrd <int> | auto                  | --HrdParametersPresent,-hrd <int> | auto                  |
| --decodedpicturehash,-dph <int> | off                     | --SEIDecodedPictureHash,-dph <int> | auto                 |

**Example usage:** Given a YUV 4:2:0 input file with a bit-depth of 8bit and a resolution of 176x144 pixels,
the following calls will encode the input file with the medium speedup preset with 1Mbit/s by using Two pass rate control. 
Both calls will produce the same output.

**standard encoder:** 

    vvencapp --preset medium -i BUS_176x144_75@15.yuv -s 176x144 -r 15 -b 1000000 -p 2 -o str.266

**full featured expert mode encoder:** 

    vvencFFapp --preset medium --InputFile BUS_176x144_75@15.yuv -s 176x144 -fr 15 -TargetBitrate 1000000 --NumPasses 2 -qpa 1 -t -1 -b str.266
    
# Contributing

Feel free to contribute. To do so:

* Fork the current-most state of the master branch
* Apply the desired changes
* Create a pull-request to the upstream repository

# License

Please see [LICENSE.txt](./LICENSE.txt) file for the terms of use of the contents of this repository.

For more information, please contact: vvc@hhi.fraunhofer.de

**Copyright (c) 2019-2021 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.**

**All rights reserved.**
