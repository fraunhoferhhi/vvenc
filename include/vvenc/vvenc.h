/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
/**
  \ingroup vvencExternalInterfaces
  \file    vvenc.h
  \brief   This file contains the external interface of the hhivvcdec SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/08/2020
*/

#pragma once

#include "stdint.h"
#include <string>
#include "../vvenc/vvencDecl.h"
#include "../vvenc/Basics.h"

namespace vvenc {


/**
  \ingroup VVEncExternalInterfaces
  \enum LogLevel
  The enum LogLevel enumerates supported log levels/verbosity.
*/
enum LogLevel
{
  LL_SILENT  = 0,
  LL_ERROR   = 1,
  LL_WARNING = 2,
  LL_INFO    = 3,
  LL_NOTICE  = 4,
  LL_VERBOSE = 5,
  LL_DETAILS = 6,
  LL_DEBUG_PLUS_INTERNAL_LOGS = 7,
};

/**
  \ingroup VVEncExternalInterfaces
  \enum Status
  The enum Status enumerates picture extra status information. The information is delivered within the AccessUnit struct and is
  related to the according picture.
*/
enum Status
{
  STATUS_NORMAL = 0,                      ///< normal
};

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
  \enum ColorFormat
  The enum ColorFormat enumerates supported input color formats.
*/
enum ColorFormat
{
  VVC_CF_INVALID       = -1,             ///< invalid color format
  VVC_CF_YUV420_PLANAR = 0,              ///< YUV420 planar color format
};

/**
  \ingroup VVEnc
  The class SliceType enumerates several supported slice types.
*/
enum VvcSliceType
{
  VVC_ST_B_SLICE               = 0,
  VVC_ST_P_SLICE               = 1,
  VVC_ST_I_SLICE               = 2,
  VVC_ST_NUMBER_OF_SLICE_TYPES = 3
};

/// supported IDR types
enum VvcDecodingRefreshType
{
  VVC_DRT_CRA                = 0,
  VVC_DRT_IDR                = 1,
  VVC_DRT_RECOVERY_POINT_SEI = 2
};

enum VvcProfile
{
  VVC_PROFILE_NONE                                 = 0,
  VVC_PROFILE_MAIN_10                              = 1,
  VVC_PROFILE_MAIN_10_444                          = 2,
  VVC_PROFILE_MAIN_10_STILL_PICTURE                = 3,
  VVC_PROFILE_MAIN_10_444_STILL_PICTURE            = 4,
  VVC_PROFILE_MULTILAYER_MAIN_10                   = 5,
  VVC_PROFILE_MULTILAYER_MAIN_10_444               = 6,
  VVC_PROFILE_MULTILAYER_MAIN_10_STILL_PICTURE     = 7,
  VVC_PROFILE_MULTILAYER_MAIN_10_444_STILL_PICTURE = 8,
  VVC_PROFILE_AUTO                                 = 9
};

enum VvcTier
{
  VVC_TIER_MAIN = 0,
  VVC_TIER_HIGH = 1,
};

enum VvcLevel
{
  VVC_LEVEL_NONE = 0,
  VVC_LEVEL_1   = 16,
  VVC_LEVEL_2   = 32,
  VVC_LEVEL_2_1 = 35,
  VVC_LEVEL_3   = 48,
  VVC_LEVEL_3_1 = 51,
  VVC_LEVEL_4   = 64,
  VVC_LEVEL_4_1 = 67,
  VVC_LEVEL_5   = 80,
  VVC_LEVEL_5_1 = 83,
  VVC_LEVEL_5_2 = 86,
  VVC_LEVEL_6   = 96,
  VVC_LEVEL_6_1 = 99,
  VVC_LEVEL_6_2 = 102,
  VVC_LEVEL_15_5 = 255,
};

enum VvcNalType
{
  VVC_NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  VVC_NAL_UNIT_CODED_SLICE_STSA,        // 1
  VVC_NAL_UNIT_CODED_SLICE_RADL,        // 2
  VVC_NAL_UNIT_CODED_SLICE_RASL,        // 3

  VVC_NAL_UNIT_RESERVED_VCL_4,
  VVC_NAL_UNIT_RESERVED_VCL_5,
  VVC_NAL_UNIT_RESERVED_VCL_6,

  VVC_NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  VVC_NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  VVC_NAL_UNIT_CODED_SLICE_CRA,         // 9
  VVC_NAL_UNIT_CODED_SLICE_GDR,         // 10

  VVC_NAL_UNIT_RESERVED_IRAP_VCL_11,
  VVC_NAL_UNIT_RESERVED_IRAP_VCL_12,

  VVC_NAL_UNIT_DCI,                     // 13
  VVC_NAL_UNIT_VPS,                     // 14
  VVC_NAL_UNIT_SPS,                     // 15
  VVC_NAL_UNIT_PPS,                     // 16
  VVC_NAL_UNIT_PREFIX_APS,              // 17
  VVC_NAL_UNIT_SUFFIX_APS,              // 18
  VVC_NAL_UNIT_PH,                      // 19
  VVC_NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  VVC_NAL_UNIT_EOS,                     // 21
  VVC_NAL_UNIT_EOB,                     // 22
  VVC_NAL_UNIT_PREFIX_SEI,              // 23
  VVC_NAL_UNIT_SUFFIX_SEI,              // 24
  VVC_NAL_UNIT_FD,                      // 25

  VVC_NAL_UNIT_RESERVED_NVCL_26,
  VVC_NAL_UNIT_RESERVED_NVCL_27,

  VVC_NAL_UNIT_UNSPECIFIED_28,
  VVC_NAL_UNIT_UNSPECIFIED_29,
  VVC_NAL_UNIT_UNSPECIFIED_30,
  VVC_NAL_UNIT_UNSPECIFIED_31,
  VVC_NAL_UNIT_INVALID
};

/**
  \ingroup VVEncExternalInterfaces
  The struct AccessUnit contains attributes that are assigned to the compressed output of the encoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of VVC encoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/

typedef struct VVENC_DECL VvcAccessUnit
{
  VvcAccessUnit()                             ///< Default constructor, sets member attributes to default values
  {}

  unsigned char*  m_pucBuffer  = nullptr;  ///< pointer to buffer that retrieves the coded data,
  int             m_iBufSize   = 0;        ///< size of the allocated buffer in bytes
  int             m_iUsedSize  = 0;        ///< length of the coded data in bytes
  uint64_t        m_uiCts      = 0;        ///< composition time stamp in TicksPerSecond (see VVCEncoderParameter)
  uint64_t        m_uiDts      = 0;        ///< decoding time stamp in TicksPerSecond (see VVCEncoderParameter)
  bool            m_bCtsValid  = false;    ///< composition time stamp valid flag (true: valid, false: CTS not set)
  bool            m_bDtsValid  = false;    ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            m_bRAP       = false;    ///< random access point flag (true: AU is random access point, false: sequential access)

  VvcSliceType    m_eSliceType = VVC_ST_NUMBER_OF_SLICE_TYPES; ///< slice type (I/P/B) */
  bool            m_bRefPic    = false;                        ///< reference picture
  int             m_iTemporalLayer  = 0;                       ///< temporal layer
  uint64_t        m_uiPOC      = 0;                            ///< picture order count

  int             m_iStatus    = 0;        ///< additional info (see Status)
  std::string     m_cInfo;                 ///< debug info from inside the encoder
} VvcAccessUnit_t;

/**
  \ingroup VVEncExternalInterfaces
  The struct PicBuffer contains attributes to hand over the uncompressed input picture and metadata related to picture. Memory has to be allocated by the user. For using maximum performance
  consider allocating 16byte aligned memory for all three color components or use HhiVvcEnc::getPreferredBuffer() to let the encoder allocate an appropriate buffer.

*/
typedef struct VVENC_DECL PicBuffer
{
  PicBuffer()                             ///< default constructor, sets member attributes to default values
  {}

  unsigned char*  m_pucDeletePicBuffer = nullptr;         ///< pointer to picture buffer origin if non zero the encoder doesn't copy the content off the buffer and deletes the buffer after encoding
                                                          ///< this implies the buffer content to be const,
                                                          ///< otherwise if the pointer is zero the buffer content is copied by the encoder into an intermediate buffer
  void*           m_pvY                = nullptr;         ///< pointer to luminance top left pixel
  void*           m_pvU                = nullptr;         ///< pointer to chrominance cb top left pixel
  void*           m_pvV                = nullptr;         ///< pointer to chrominance cbr top left pixel
  int             m_iWidth             = 0;               ///< width of the luminance plane
  int             m_iHeight            = 0;               ///< height of the luminance plane
  int             m_iStride            = 0;               ///< stride (width + left margin + right margins) of luminance plane chrominance stride is assumed to be stride/2
  int             m_iCStride           = 0;               ///< stride (width + left margin + right margins) of chrominance plane in case its value differs from stride/2
  int             m_iBitDepth          = 0;               ///< bit depth of input signal (8: depth 8 bit, 10: depth 10 bit  )
  ColorFormat     m_eColorFormat       = VVC_CF_INVALID;  ///< color format (VVC_CF_YUV420_PLANAR)
  uint64_t        m_uiSequenceNumber   = 0;               ///< sequence number of the picture
  uint64_t        m_uiCts              = 0;               ///< composition time stamp in TicksPerSecond (see VVCEncoderParameter)
  bool            m_bCtsValid          = false;           ///< composition time stamp valid flag (true: valid, false: CTS not set)
} PicBuffer_t;

/**
  \ingroup VVEncExternalInterfaces
  The struct PicAttributes - currently not used.
*/
typedef struct VVENC_DECL PicAttributes
{
  PicAttributes() {}
} PicAttributes_t;


/**
  \ingroup VVEncExternalInterfaces
  The struct InputPicture combines the struct PicBuffer and the optional PicAttributes class.
*/
typedef struct VVENC_DECL InputPicture
{
  InputPicture() {}
  PicBuffer       m_cPicBuffer;                 ///< instance of PicBuffer, that holds input picture and related meta information
  PicAttributes*  m_pcPicAttributes = nullptr;  ///< pointer to PicAttribute that might be NULL, containing encoder side information
} InputPicture_t;

/**
  \ingroup VVEncExternalInterfaces
  The struct VVEncParameter is a container for encoder configuration parameters. This struct is used for initialization of an blank encoder
  as well as for reconfiguration of an already initialized encoder. The struct is equipped with an default constructor that initializes all parameters
  to default values for ease of use and best performance. However, some of the parameters has to be set by the caller, which can not be guessed by the encoder.
*/
typedef struct VVENC_DECL VVEncParameter
{
  VVEncParameter()           ///< default constructor, sets member attributes to default values
  {}
  int m_iQp                   = 30;     ///< quantization parameter                                 (no default || 0-51)
  int m_iWidth                = 0;      ///< luminance width of input picture                       (no default || 2..4096)
  int m_iHeight               = 0;      ///< luminance height of input picture                      (no default || 2/4..2160)
  int m_iGopSize              = 16;     ///< gop size                                               (default: 16 || 1: low delay, 16,32: hierarchical b frames)
  VvcDecodingRefreshType m_eDecodingRefreshType = VVC_DRT_IDR; ///< intra period refresh type            (default: VVC_DRT_IDR )
  int m_iIDRPeriodSec         = 1;       ///< intra period for IDR/CRA intra refresh/RAP flag in seconds (default: 1 || -1: only the first pic, otherwise refresh in seconds
  int m_iIDRPeriod            = 0;       ///< intra period for IDR/CRA intra refresh/RAP flag in frames  (default: 0 || -1: only the first pic, otherwise factor of m_iGopSize
  LogLevel m_eLogLevel        = LL_INFO; ///< log level                                             (default: 0 || 0: no logging,  > 4 (LL_VERBOSE,LL_DETAILS)enables psnr/rate output  0: silent, 1: error, 2: warning, 3: info, 4: notice: 5, verbose, 6: details
  int m_iTemporalRate         = 60;     ///< temporal rate /numerator for fps                       (no default || e.g. 50, 60000 -> 1-60 fps)
  int m_iTemporalScale        = 1;      ///< temporal scale /denominator for fps                    (no default || 1, 1001)
  int m_iTicksPerSecond       = 90000;  ///< ticks per second e.g. 90000 for dts generation         (no default || 1..27000000)
  int m_iThreadCount          = 1;      ///< number of worker threads (no default || should not exceed the number of physical cpu's)
  int m_iQuality              = 2;      ///< encoding quality vs speed                              (no default || 2    0: faster, 1: fast, 2: medium, 3: slow
  int m_iPerceptualQPA        = 0;      ///< perceptual qpa usage                                   (default: 0 || Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = SDR WPSNR based, 2 = SDR XPSNR based, 3 = HDR WPSNR based, 4 = HDR XPSNR based, 5 = HDR mean-luma based))
  int m_iTargetBitRate        = 0;      ///< target bit rate in bps                                 (no default || 0 : VBR, otherwise bitrate [bits per sec]
  VvcProfile m_eProfile       = VVC_PROFILE_MAIN_10; ///< vvc profile                               (default: main_10)
  VvcLevel m_eLevel           = VVC_LEVEL_5_1;       ///< vvc level_idc                             (default: 5.1 )
  VvcTier  m_eTier            = VVC_TIER_MAIN;       ///< vvc tier                                  (default: main )
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
    \param[in]  pcInputPicture pointer to InputPicture structure containing uncompressed picture data and meta information, if pcInputPicture is NULL the encoder only checks for pending output data and returns a chunk if available.
    \param[out] rcAccessUnit reference to AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
    \retval     int if non-zero an error occurred, otherwise the retval indicates success VVENC_OK
    \pre        The encoder has to be initialized successfully.
  */
   int encode( InputPicture* pcInputPicture, VvcAccessUnit& rcVvcAccessUnit);


   /**
     This method flushes the encoder. Use this method if a specific number of frames has to be encoded.
     This call is used to get outstanding output data after all input frames have been passed over into the encoder using the encode call.
     Using the flush method the encoder is signaled that there are no further input pictures to encode.
     The caller should repeat the flush call until all pending output packets has been delivered to the caller, which is when the UsedSize attribute in the AccessUnit struct gets zero.
     If the call returns VVENC_NOT_ENOUGH_MEM, the BufSize attribute in AccessUnit struct indicates that the buffer is to small to retrieve the compressed data waiting for delivery.
     In this case the UsedSize attribute returns the minimum buffersize required to fetch the pending chunk. After allocating sufficient memory the encoder can retry fetching the outstanding chunks.
     \param[out] rcAccessUnit reference to AccessUnit
     \retval     int if non-zero an error occurred, otherwise the retval indicates success VVENC_OK
     \pre        The encoder has to be initialized.
   */
   int flush( VvcAccessUnit& rcVvcAccessUnit );

  /**
    This method returns an allocated picture buffer according to the encoder's preference. To is this call the encoder has to be initialized.
    \param[out] rcPicBuffer reference to PicBuffer
    \retval     int nonzero indicates an error VVENC_ERR_INITIALIZE, VVENC_ERR_ALLOCATE, otherwise VVENC_OK
     \pre       The encoder has to be initialized.
  */
   int getPreferredBuffer( PicBuffer &rcPicBuffer );

   /**
     This method sets a encoder start timer.
   */
   void clockStartTime();

   /**
     This method sets a encoder finish timer.
   */
   void clockEndTime();

   /**
     This method return the clock difference of end and start time.
     \retval     double clock time difference in milli seconds
   */
   double clockGetTimeDiffMs();

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
     become invalid after the parameter change. If changes are limited to TargetBitRate, QP or LogLevel changes then the encoder continues encoding
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
   const char* getLastError() const;


   const char* getEncoderInfo() const;

   /**
     This method returns the encoder version number as a string.
     \param      None
     \retval     std::string returns the version number
   */
   static const char* getVersionNumber();

   /**
     This static function returns a string according to the passed parameter nRet.
     \param[in]  nRet return value code to translate
     \retval[ ]  std::string empty string for no error
   */
   static const char* getErrorMsg( int nRet );

   /**
     This static function returns a string according to the passed parameter iQuality.
     \param[in]  iQuality Quality (preset) as integer
     \retval[ ]  std::string enabled encoding parameter as string
   */
   static const char* getPresetParamsAsStr( int iQuality );

private:
   VVEncImpl*  m_pcVVEncImpl;
};


} // namespace

