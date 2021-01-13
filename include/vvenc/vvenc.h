/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
/**
  \ingroup vvencExternalInterfaces
  \file    vvenc.h
  \brief   This file contains the external interface of the hhivvcdec SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/08/2020
*/

#pragma once

#include <functional>
#include "stdint.h"
#include <string>
#include "vvenc/vvencDecl.h"


#include "vvenc/EncCfg.h"
#include "vvenc/vvencConfig.h"

namespace vvenc {


/**
  \ingroup VVEncExternalInterfaces
  \enum ErrorCodes
  The enum ErrorCodes enumerates error codes returned by the encoder.
*/
enum ErrorCodes
{
  VVENC_OK                   = 0,      ///< success
  VVENC_ERR_UNSPECIFIED      = -1,     ///< unspecified malfunction
  VVENC_ERR_INITIALIZE       = -2,     ///< encoder not initialized or tried to initialize multiple times
  VVENC_ERR_ALLOCATE         = -3,     ///< internal allocation error
  VVENC_NOT_ENOUGH_MEM       = -5,     ///< allocated memory to small to receive encoded data. After allocating sufficient memory the failed call can be repeated.
  VVENC_ERR_PARAMETER        = -7,     ///< inconsistent or invalid parameters
  VVENC_ERR_NOT_SUPPORTED    = -10,    ///< unsupported request
  VVENC_ERR_RESTART_REQUIRED = -11,    ///< encoder requires restart
  VVENC_ERR_CPU              = -30     ///< unsupported CPU SSE 4.1 needed
};

/**
  \ingroup VVEncExternalInterfaces
  The struct PicBuffer contains attributes to hand over the uncompressed input picture and metadata related to picture. Memory has to be allocated by the user. For using maximum performance
  consider allocating 16byte aligned memory for all three color components.
*/
typedef struct VVENC_DECL YuvPicture
{
  unsigned char*  deletePicBuffer  = nullptr;         ///< pointer to picture buffer origin if non zero the encoder doesn't copy the content off the buffer and deletes the buffer after encoding
                                                      ///< this implies the buffer content to be const,
                                                      ///< otherwise if the pointer is zero the buffer content is copied by the encoder into an intermediate buffer
  void*           y                 = nullptr;        ///< pointer to luminance top left pixel
  void*           u                 = nullptr;        ///< pointer to chrominance cb top left pixel
  void*           v                 = nullptr;        ///< pointer to chrominance cbr top left pixel
  int             width             = 0;              ///< width of the luminance plane
  int             height            = 0;              ///< height of the luminance plane
  int             stride            = 0;              ///< stride (width + left margin + right margins) of luminance plane chrominance stride is assumed to be stride/2
  int             cStride           = 0;              ///< stride (width + left margin + right margins) of chrominance plane in case its value differs from stride/2
  int             bitDepth          = 10;             ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  ColorFormat     colorFormat       = VVC_CF_INVALID; ///< color format (VVC_CF_YUV420_PLANAR)
  uint64_t        sequenceNumber    = 0;               ///< sequence number of the picture
  uint64_t        cts               = 0;               ///< composition time stamp in TicksPerSecond (see VVCEncoderParameter)
  bool            ctsValid          = false;          ///< composition time stamp valid flag (true: valid, false: CTS not set)
} YuvPicture_t;


// will be removed  CL
struct VVENC_DECL YUVPlane
{
  int16_t*   planeBuf = nullptr;  ///< pointer to plane buffer
  int     width    = 0;        ///< width of the plane
  int     height   = 0;        ///< height of the plane
  int     stride   = 0;        ///< stride (width + left margin + right margins) of plane in samples
};

struct VVENC_DECL YUVBuffer
{
  YUVPlane  yuvPlanes[ MAX_NUM_COMP ];
  uint64_t  sequenceNumber  = 0;      ///< sequence number of the picture
  uint64_t  cts             = 0;      ///< composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  bool      ctsValid        = false;  ///< composition time stamp valid flag (true: valid, false: CTS not set)
};

// ----------------------------------------

class VVENC_DECL YUVWriterIf
{
protected:
  YUVWriterIf() {}
  virtual ~YUVWriterIf() {}

public:
  virtual void outputYuv( const YUVBuffer& /*yuvOutBuf*/ )
  {
  }

  virtual void outputYuv( const YuvPicture& /*yuvOutBuf*/ )
  {
  }
};


/**
  \ingroup VVEncExternalInterfaces
  The struct AccessUnitList contains attributes that are assigned to the compressed output of the encoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of VVC encoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/

typedef struct VVENC_DECL AccessUnit
{
  std::vector<uint8_t> payload;
  uint64_t        cts           = 0;        ///< composition time stamp in TicksPerSecond (see VVCEncoderParameter)
  uint64_t        dts           = 0;        ///< decoding time stamp in TicksPerSecond (see VVCEncoderParameter)
  bool            ctsValid      = false;    ///< composition time stamp valid flag (true: valid, false: CTS not set)
  bool            dtsValid      = false;    ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            rap           = false;    ///< random access point flag (true: AU is random access point, false: sequential access)
  SliceType       sliceType     = NUMBER_OF_SLICE_TYPES; ///< slice type (I/P/B) */
  bool            refPic        = false;    ///< reference picture
  int             temporalLayer = 0;        ///< temporal layer
  uint64_t        poc           = 0;        ///< picture order count

  int             status        = 0;        ///< additional info (see Status)
  std::string     infoString;               ///< debug info from inside the encoder

  std::vector<NalUnitType> nalUnitTypeVec;
  std::vector<uint32_t>    annexBsizeVec;

} AccessUnit_t;

/**
  \ingroup VVEncExternalInterfaces
  The struct VVEncParameter is a container for encoder configuration parameters. This struct is used for initialization of an blank encoder
  as well as for reconfiguration of an already initialized encoder. The struct is equipped with an default constructor that initializes all parameters
  to default values for ease of use and best performance. However, some of the parameters has to be set by the caller, which can not be guessed by the encoder.
*/
typedef struct VVENC_DECL VVEncParameter
{
  int qp                   = 32;     ///< quantization parameter                                 (no default || 0-51)
  int width                = 0;      ///< luminance width of input picture                       (no default || 2..4096)
  int height               = 0;      ///< luminance height of input picture                      (no default || 2/4..2160)
  int gopSize              = 32;     ///< gop size                                               (default: 16 || 1: low delay, 16,32: hierarchical b frames)
  DecodingRefreshType decodingRefreshType = DRT_IDR; ///< intra period refresh type           (default: VVC_DRT_IDR )
  int idrPeriodSec         = 1;       ///< intra period for IDR/CRA intra refresh/RAP flag in seconds (default: 1 || -1: only the first pic, otherwise refresh in seconds
  int idrPeriod            = 0;       ///< intra period for IDR/CRA intra refresh/RAP flag in frames  (default: 0 || -1: only the first pic, otherwise factor of m_iGopSize
  MsgLevel msgLevel        = INFO; ///< log level                                             (default: 0 || 0: no logging,  > 4 (LL_VERBOSE,LL_DETAILS)enables psnr/rate output  0: silent, 1: error, 2: warning, 3: info, 4: notice: 5, verbose, 6: details
  int temporalRate         = 60;     ///< temporal rate /numerator for fps                       (no default || e.g. 50, 60000 -> 1-60 fps)
  int temporalScale        = 1;      ///< temporal scale /denominator for fps                    (no default || 1, 1001)
  int ticksPerSecond       = 90000;  ///< ticks per second e.g. 90000 for dts generation         (no default || 1..27000000)
  int maxFrames            = 0;      ///< max number of frames to be encoded                     (default 0: encode all frames)
  int frameSkip            = 0;      ///< number of frames to skip before start encoding         (default 0: off)
  int threadCount          = -1;     ///< number of worker threads (no default || should not exceed the number of physical cpu's)
  int quality              = 2;      ///< encoding quality vs speed                              (no default || 2    0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  int perceptualQPA        = 2;      ///< perceptual qpa usage                                   (default: 0 || Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = SDR WPSNR based, 2 = SDR XPSNR based, 3 = HDR WPSNR based, 4 = HDR XPSNR based, 5 = HDR mean-luma based))
  int targetBitRate        = 0;      ///< target bit rate in bps                                 (no default || 0 : VBR, otherwise bitrate [bits per sec]
  int numPasses            = 1;      ///< number of rate control passes                          (default: 1)
  int inputBitDepth        = 8;      ///< input bit-depth                                        (default: 8)
  int internalBitDepth     = 10;     ///< internal bit-depth                                     (default: 10)
  Profile profile          = Profile::MAIN_10; ///< vvc profile                            (default: main_10)
  Level   level            = Level::LEVEL5_1;  ///< vvc level_idc                          (default: 5.1)
  Tier    tier             = Tier::TIER_MAIN;      ///< vvc tier                               (default: main)
  SegmentMode segmentMode  = SEG_OFF;               ///< segment mode                            (default: off)
  bool useAccessUnitDelimiter       = false;  ///< enable aud                                       (default: off)
  bool useHrdParametersPresent      = false;  ///< enable hrd                                       (default: off)
  bool useBufferingPeriodSEIEnabled = false;  ///< enable bp sei                                    (default: off)
  bool usePictureTimingSEIEnabled   = false;  ///< enable pt sei                                    (default: off)
} VVEncParameter_t;


class VVEncImpl;

/**
  \ingroup VVEncExternalInterfaces
  The class HhiVvcEnc provides the encoder's user interface. The simplest way to use the encoder is to call init() to initialize an encoder instance with the
  the given VVCEncoderParameters. After initialization the encoding of the video is performed by using the encode() method to hand over frame by frame in display order
  and retrieve the compressed bitstream chunks of already processed pictures. The encoding can be end by calling flush() that causes the encoder to finish encoding of all pending pictures.
  Finally calling uninit() releases all allocated resources held by the encoder internally.
  Beside the basic functionality of encoding there are some more methods available.
  For instance, the reconfig() is used to reconfigure a running encoder. There are also method providing some service functionality like getting or printing encoder configuration,
  or getting information about the encoder or it's current state.
  The HhiVvcEnc also provides a static call that used the encoder as remote support client for joint encoding on two machines.
*/
class VVENC_DECL VVEnc
{
public:
  /// Constructor
  VVEnc();

  /// Destructor
  virtual ~VVEnc();

public:
  /**
    This method initializes the encoder instance.
    This method is used to initially set up the encoder with the assigned encoder parameter struct.
    The method fails if the encoder is already initialized or if the assigned parameter struct
    does not pass the consistency check. Other possibilities for an unsuccessful call are missing encoder license, or an machine with
    insufficient CPU-capabilities.
    \param[in]  rvVVEncParameter const reference of VVEncParameter struct that holds initial encoder parameters.
    \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVENC_OK
    \pre        The encoder must not be initialized.
  */
   int init( const VVEncParameter& rcVVEncParameter );

   int init( const EncCfg& rcEncCfg, YUVWriterIf* YUVWriterIf = nullptr );
  /**
    This method initializes the encoder instance in dependency to the encoder pass.
  */
   int initPass( int pass );

   /**
    This method resets the encoder instance.
    This method clears the encoder and releases all internally allocated memory.
    Calling uninit cancels all pending encoding calls. In order to finish pending input pictures use the flush method.
    \param[in]  None
    \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
    \pre        None
  */
   int uninit();

   bool isInitialized();

  /**
    This method encodes a picture.
    Uncompressed input pictures are passed to the encoder in display order. A compressed bitstream chunks is returned by filling the assigned AccessUnit struct.
    Data in AcccessUnit struct are valid if the encoder call returns success and the UsedSize attribute is non-zero.
    If the input parameter pcInputPicture is NULL, the encoder just returns a pending bitstream chunk if available.
    If the call returns VVENC_NOT_ENOUGH_MEM, the BufSize attribute in AccessUnit struct indicates that the buffer is to small to retrieve the compressed data waiting for delivery.
    In this case the UsedSize attribute returns the minimum buffersize required to fetch the pending chunk. After allocating sufficient memory the encoder can retry the last call with the parameter pcInputPicture set to NULL to prevent encoding the last picture twice.
    \param[in]  pcYuvPicture pointer to InputPicture structure containing uncompressed picture data and meta information, to flush the encoder pcYuvPicture must be NULL.
    \param[out] rcAccessUnit reference to AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
    \param[out] rbEncodeDone reference to flag that indicates that the encoder completed the last frame after flushing.
    \retval     int if non-zero an error occurred, otherwise the retval indicates success VVENC_OK
    \pre        The encoder has to be initialized successfully.
  */
   int encode( YuvPicture* pcYuvPicture, AccessUnit& rcAccessUnit, bool& rbEncodeDone);

   int encode( YUVBuffer* pcYUVBuffer, AccessUnit& rcAccessUnit, bool& rbEncodeDone);

   /**
     This method fetches the current encoder configuration.
     The method fails if the encoder is not initialized.
     \param[in]  rcVVCEncoderParameter reference to an VVCEncoderParameter struct that returns the current encoder setup.
     \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
     \pre        The encoder has to be initialized.
   */
   int getConfig( VVEncParameter& rcVVEncParameter );

    /**
     This method reconfigures the encoder instance.
     This method is used to change encoder settings during the encoding process when the encoder was already initialized.
     Some parameter changes might require an internal encoder restart, especially when previously used parameter sets VPS, SPS or PPS
     become invalid after the parameter change. If changes are limited to TargetBitRate or QP changes then the encoder continues encoding
     without interruption, using the new parameters. Some parameters e.g. NumTheads are not reconfigurable - in this case the encoder returns an Error.
     The method fails if the encoder is not initialized or if the assigned parameter set given in VVCEncoderParameter struct
     does not pass the consistency and parameter check.
     \param[in]  rcVVCEncoderParameter const reference to VVCEncoderParameter struct that holds the new encoder parameters.
     \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
     \pre        The encoder has to be initialized successfully.
   */
   int reconfig( const VVEncParameter& rcVVEncParameter );

   /**
     This method checks the passed configuration.
     The method fails if the encoder is not initialized.
     \param[in]  rcVVCEncParameter reference to an VVCEncParameter struct that returns the current encoder setup.
     \retval     int VVENC_ERR_PARAMETER indicates a parameter error, otherwise the return value VVENC_OK indicates success.
   */
   int checkConfig( const VVEncParameter& rcVVEncParameter );

    /**
     This method returns the last occurred error as a string.
     \param      None
     \retval     std::string empty string for no error assigned
   */
   std::string getLastError() const;

   std::string getEncoderInfo() const;

   int getNumLeadFrames() const;

   int getNumTrailFrames() const;

   int printConfig() const;
   int printSummary() const;

   /**
     This method returns the encoder version number as a string.
     \param      None
     \retval     std::string returns the version number
   */
   static std::string getVersionNumber();

   /**
     This static function returns a string according to the passed parameter nRet.
     \param[in]  nRet return value code to translate
     \retval[ ]  std::string empty string for no error
   */
   static std::string getErrorMsg( int nRet );

   /**
     This static function returns a string according to the passed parameter iQuality.
     \param[in]  iQuality Quality (preset) as integer
     \retval[ ]  std::string enabled encoding parameter as string
   */
   static std::string getPresetParamsAsStr( int iQuality );

   /**
     This method registers a log message callback function to the encoder library. 
     If no such function has been registered, the library will omit all messages.
     \param      Log message callback function.
   */
   static void registerMsgCbf( std::function<void( int, const char*, va_list )> msgCbf );

   ///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
   static std::string setSIMDExtension( const std::string& simdId );
   ///< checks if library has tracing supported enabled (see ENABLE_TRACING).
   static bool        isTracingEnabled();
   ///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
   static std::string getCompileInfoString();
   ///< decode bitstream with limited build in decoder
   static void        decodeBitstream( const std::string& FileName);


private:
   VVEncImpl*  m_pcVVEncImpl;
};



} // namespace

