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
  The struct VVEncParameter is a container for encoder configuration parameters. This struct is used for initialization of an blank encoder
  as well as for reconfiguration of an already initialized encoder. The struct is equipped with an default constructor that initializes all parameters
  to default values for ease of use and best performance. However, some of the parameters has to be set by the caller, which can not be guessed by the encoder.
*/
typedef struct VVENC_DECL VVEncParameter
{
  VVEncParameter()           ///< default constructor, sets member attributes to default values
  {}
  int m_iQp                   = 32;     ///< quantization parameter                                 (no default || 0-51)
  int m_iWidth                = 0;      ///< luminance width of input picture                       (no default || 2..4096)
  int m_iHeight               = 0;      ///< luminance height of input picture                      (no default || 2/4..2160)
  int m_iGopSize              = 32;     ///< gop size                                               (default: 16 || 1: low delay, 16,32: hierarchical b frames)
  VvcDecodingRefreshType m_eDecodingRefreshType = DRT_IDR; ///< intra period refresh type           (default: VVC_DRT_IDR )
  int m_iIDRPeriodSec         = 1;       ///< intra period for IDR/CRA intra refresh/RAP flag in seconds (default: 1 || -1: only the first pic, otherwise refresh in seconds
  int m_iIDRPeriod            = 0;       ///< intra period for IDR/CRA intra refresh/RAP flag in frames  (default: 0 || -1: only the first pic, otherwise factor of m_iGopSize
  MsgLevel m_eMsgLevel        = INFO; ///< log level                                             (default: 0 || 0: no logging,  > 4 (LL_VERBOSE,LL_DETAILS)enables psnr/rate output  0: silent, 1: error, 2: warning, 3: info, 4: notice: 5, verbose, 6: details
  int m_iTemporalRate         = 60;     ///< temporal rate /numerator for fps                       (no default || e.g. 50, 60000 -> 1-60 fps)
  int m_iTemporalScale        = 1;      ///< temporal scale /denominator for fps                    (no default || 1, 1001)
  int m_iTicksPerSecond       = 90000;  ///< ticks per second e.g. 90000 for dts generation         (no default || 1..27000000)
  int m_iMaxFrames            = 0;      ///< max number of frames to be encoded                     (default 0: encode all frames)
  int m_iFrameSkip            = 0;      ///< number of frames to skip before start encoding         (default 0: off)
  int m_iThreadCount          = -1;     ///< number of worker threads (no default || should not exceed the number of physical cpu's)
  int m_iQuality              = 2;      ///< encoding quality vs speed                              (no default || 2    0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  int m_iPerceptualQPA        = 2;      ///< perceptual qpa usage                                   (default: 0 || Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = SDR WPSNR based, 2 = SDR XPSNR based, 3 = HDR WPSNR based, 4 = HDR XPSNR based, 5 = HDR mean-luma based))
  int m_iTargetBitRate        = 0;      ///< target bit rate in bps                                 (no default || 0 : VBR, otherwise bitrate [bits per sec]
  int m_iNumPasses            = 1;      ///< number of rate control passes                          (default: 1) 
  int m_iInputBitDepth        = 8;      ///< input bit-depth                                        (default: 8)
  int m_iInternalBitDepth     = 10;     ///< internal bit-depth                                     (default: 10)
  Profile::Name m_eProfile    = Profile::Name::MAIN_10; ///< vvc profile                            (default: main_10)
  Level::Name   m_eLevel      = Level::Name::LEVEL5_1;  ///< vvc level_idc                          (default: 5.1)
  Level::Tier   m_eTier       = Level::Tier::MAIN;      ///< vvc tier                               (default: main)
  SegmentMode   m_eSegMode    = SEG_OFF;               ///< segment mode                            (default: off)
  bool m_bAccessUnitDelimiter       = false;  ///< enable aud                                       (default: off)
  bool m_bHrdParametersPresent      = false;  ///< enable hrd                                       (default: off)
  bool m_bBufferingPeriodSEIEnabled = false;  ///< enable bp sei                                    (default: off)
  bool m_bPictureTimingSEIEnabled   = false;  ///< enable pt sei                                    (default: off)
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

   int init( const EncCfg& rcEncCfg );
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

private:
   VVEncImpl*  m_pcVVEncImpl;
};


} // namespace

