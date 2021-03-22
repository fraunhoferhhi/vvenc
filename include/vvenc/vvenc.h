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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
  \file    vvenc.h
  \brief   This file contains the external interface of the vvenc SDK.
*/

#ifndef _VVENC_H_
#define _VVENC_H_

#include "vvenc/vvencDecl.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "vvenc/vvencCfg.h"

#define VVENC_NAMESPACE_BEGIN
#define VVENC_NAMESPACE_END

#ifdef __cplusplus
extern "C" {
#endif

VVENC_NAMESPACE_BEGIN


/**
  The class HhiVvcEnc provides the encoder's user interface. The simplest way to use the encoder is to call init() to initialize an encoder instance with the
  the given VVEncCfg. After initialization the encoding of the video is performed by using the encode() method to hand over frame by frame in display order
  and retrieve the compressed bitstream chunks of already processed pictures. The encoding can be end by calling flush() that causes the encoder to finish encoding of all pending pictures.
  Finally calling uninit() releases all allocated resources held by the encoder internally.
  Beside the basic functionality of encoding there are some more methods available.
  For instance, the reconfig() is used to reconfigure a running encoder. There are also method providing some service functionality like getting or printing encoder configuration,
  or getting information about the encoder or it's current state.
  The HhiVvcEnc also provides a static call that used the encoder as remote support client for joint encoding on two machines.
*/


/* vvencEncoder:
 * opaque handler for the decoder */
typedef struct vvencEncoder vvencEncoder;

/* vvdecLoggingCallback:
   callback function to receive messages of the encoder library
*/
typedef void (*vvencLoggingCallback)(void*, int, const char*, va_list);

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
  The struct YUVBuffer contains attributes to hand over the uncompressed input picture and metadata related to picture.
*/

typedef struct vvencYUVBuffer
{
  struct vvencPlane
  {
    int16_t*  ptr     = nullptr;      ///< pointer to plane buffer
    int       width   = 0;            ///< width of the plane
    int       height  = 0;            ///< height of the plane
    int       stride  = 0;            ///< stride (width + left margin + right margins) of plane in samples
  };

  vvencPlane planes[ 3 ];
  uint64_t   sequenceNumber  = 0;      ///< sequence number of the picture
  uint64_t   cts             = 0;      ///< composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  bool       ctsValid        = false;  ///< composition time stamp valid flag (true: valid, false: CTS not set)
}vvencYUVBuffer;

typedef void (*vvencYUVWriterCallback)(void*, vvencYUVBuffer* );

VVENC_DECL vvencYUVBuffer* vvenc_YUVBuffer_alloc();
VVENC_DECL void vvenc_YUVBuffer_free(vvencYUVBuffer *yuvBuffer );
VVENC_DECL void vvenc_YUVBuffer_default(vvencYUVBuffer *yuvBuffer );


VVENC_DECL vvencYUVBuffer* vvenc_YUVBuffer_alloc_buffer( vvencYUVBuffer *yuvBuffer, const vvencChromaFormat& chFmt, const int frameWidth, const int frameHeight );
VVENC_DECL vvencYUVBuffer* vvenc_YUVBuffer_free_buffer( vvencYUVBuffer *yuvBuffer );


// ----------------------------------------



/**
  \ingroup VVEncExternalInterfaces
  The struct AccessUnit contains attributes that are assigned to the compressed output of the encoder for a specific input picture.
  The structure contains buffer and size information of the compressed payload as well as timing, access and debug information.
  The smallest output unit of VVC encoders are NalUnits. A set of NalUnits that belong to the same access unit are delivered in a continuous bitstream,
  where the NalUnits are separated by three byte start codes.
  The Buffer to retrieve the compressed video chunks has to be allocated by the caller. The related attribute BufSize
*/
typedef struct vvencAccessUnit
{
  unsigned char*  payload;             // pointer to buffer that retrieves the coded data,
  int             payloadSize;         // size of the allocated buffer in bytes
  int             payloadUsedSize;     // length of the coded data in bytes

  uint64_t        cts           = 0;        ///< composition time stamp in TicksPerSecond (see VVEncCfg)
  uint64_t        dts           = 0;        ///< decoding time stamp in TicksPerSecond (see VVEncCfg)
  bool            ctsValid      = false;    ///< composition time stamp valid flag (true: valid, false: CTS not set)
  bool            dtsValid      = false;    ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            rap           = false;    ///< random access point flag (true: AU is random access point, false: sequential access)
  vvencSliceType  sliceType     = VVENC_NUMBER_OF_SLICE_TYPES; ///< slice type (I/P/B) */
  bool            refPic        = false;    ///< reference picture
  int             temporalLayer = 0;        ///< temporal layer
  uint64_t        poc           = 0;        ///< picture order count

  int             status        = 0;        ///< additional info (see Status)
  std::string     infoString;               ///< debug info from inside the encoder

//  std::vector<vvencNalUnitType> nalUnitTypeVec;
//  std::vector<uint32_t>    annexBsizeVec;

} vvencAccessUnit;


VVENC_DECL vvencAccessUnit* vvenc_accessUnit_alloc();

/* vvdec_accessUnit_free:
   release storage of an vvdecAccessUnit instance.
   The payload memory is also released if not done yet.
*/
VVENC_DECL void vvenc_accessUnit_free(vvencAccessUnit *accessUnit );

/* vvdec_accessUnit_alloc_payload:
   Allocates the memory for an accessUnit payload.
   To free the memory use vvdecAccessUnit_free_payload.
   When the vvdecAccessUnit memory is released the payload memory is also released.
*/
VVENC_DECL void vvenc_accessUnit_alloc_payload(vvencAccessUnit *accessUnit, int payload_size );

/* vvdec_accessUnit_free_payload:
   release storage of the payload in an vvdecAccessUnit instance.
*/
VVENC_DECL void vvenc_accessUnit_free_payload(vvencAccessUnit *accessUnit );

/* vvdec_accessUnit_default:
  Initialize vvdecAccessUnit structure to default values
*/
VVENC_DECL void vvenc_accessUnit_default(vvencAccessUnit *accessUnit );


/**
  This method initializes the encoder instance.
  This method is used to initially set up the encoder with the assigned encoder parameter struct.
  The method fails if the encoder is already initialized or if the assigned parameter struct
  does not pass the consistency check. Other possibilities for an unsuccessful call are missing encoder license, or an machine with
  insufficient CPU-capabilities.
  \param[in]  rcVVEncCfg const reference of VVEncCfg struct that holds initial encoder parameters.
  \param[in]  ptrYUVWriterIf pointer to callback interface YUVWriteIf used to emit reconstruced samples.
  \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVENC_OK
  \pre        The encoder must not be initialized.
*/
VVENC_DECL vvencEncoder* vvenc_encoder_open( VVEncCfg* );

/**
 This method resets the encoder instance.
 This method clears the encoder and releases all internally allocated memory.
 Calling uninit cancels all pending encoding calls. In order to finish pending input pictures use the flush method.
 \param[in]  None
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        None
*/
VVENC_DECL int vvenc_encoder_close(vvencEncoder *);


VVENC_DECL int vvenc_encoder_setYUVWriterCallback(vvencEncoder *, vvencYUVWriterCallback callback );

/**
  This method initializes the encoder instance in dependency to the encoder pass.
*/
VVENC_DECL int vvenc_initPass( vvencEncoder *, int pass );

/**
  This method encodes a picture.
  Uncompressed input pictures are passed to the encoder in display order. A compressed bitstream chunks is returned by filling the assigned AccessUnit struct.
  Data in AcccessUnit struct are valid if the encoder call returns success and the UsedSize attribute is non-zero.
  If the input parameter pcInputPicture is NULL, the encoder just returns a pending bitstream chunk if available.
  If the call returns VVENC_NOT_ENOUGH_MEM, the BufSize attribute in AccessUnit struct indicates that the buffer is to small to retrieve the compressed data waiting for delivery.
  In this case the UsedSize attribute returns the minimum buffersize required to fetch the pending chunk. After allocating sufficient memory the encoder can retry the last call with the parameter pcInputPicture set to NULL to prevent encoding the last picture twice.
  \param[in]  pcYUVBuffer pointer to YUVBuffer structure containing uncompressed picture data and meta information, to flush the encoder pcYUVBuffer must be NULL.
  \param[out] rcAccessUnit reference to AccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
  \param[out] rbEncodeDone reference to flag that indicates that the encoder completed the last frame after flushing.
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVENC_OK
  \pre        The encoder has to be initialized successfully.
*/
VVENC_DECL int vvenc_encode( vvencEncoder *, vvencYUVBuffer* YUVBuffer, vvencAccessUnit** accessUnit, bool* encodeDone );

/**
 This method fetches the current encoder configuration.
 The method fails if the encoder is not initialized.
 \param[in]  rcVVEncCfg reference to an VVEncCfg struct that returns the current encoder setup.
 \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
 \pre        The encoder has to be initialized.
*/
VVENC_DECL int getConfig( vvencEncoder *,VVEncCfg& rcVVEncCfg );

/**
 This method reconfigures the encoder instance.
 This method is used to change encoder settings during the encoding process when the encoder was already initialized.
 Some parameter changes might require an internal encoder restart, especially when previously used parameter sets VPS, SPS or PPS
 become invalid after the parameter change. If changes are limited to TargetBitRate or QP changes then the encoder continues encoding
 without interruption, using the new parameters. Some parameters e.g. NumTheads are not reconfigurable - in this case the encoder returns an Error.
 The method fails if the encoder is not initialized or if the assigned parameter set given in VVEncCfg struct
 does not pass the consistency and parameter check.
 \param[in]  rcVVEncCfg const reference to VVEncCfg struct that holds the new encoder parameters.
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        The encoder has to be initialized successfully.
*/
VVENC_DECL int reconfig( vvencEncoder *,VVEncCfg* );

/**
 This method checks the passed configuration.
 The method fails if the encoder is not initialized.
 \param[in]  rcVVCEncParameter reference to an VVCEncParameter struct that returns the current encoder setup.
 \retval     int VVENC_ERR_PARAMETER indicates a parameter error, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int checkConfig( vvencEncoder *,VVEncCfg* );

/**
 This method returns the last occurred error as a string.
 \param      None
 \retval     std::string empty string for no error assigned
*/
VVENC_DECL std::string vvenc_getLastError()

VVENC_DECL std::string vvenc_getEncoderInfo()

VVENC_DECL int vvenc_getNumLeadFrames()

VVENC_DECL int vvenc_getNumTrailFrames()

VVENC_DECL int vvenc_printSummary()

/**
 This method returns the encoder version number as a string.
 \param      None
 \retval     std::string returns the version number
*/
VVENC_DECL std::string vvenc_getVersionNumber();

/**
 This static function returns a string according to the passed parameter nRet.
 \param[in]  nRet return value code to translate
 \retval[ ]  std::string empty string for no error
*/
VVENC_DECL std::string vvenc_getErrorMsg( int nRet );

/**
 This method registers a log message callback function to the encoder library.
 If no such function has been registered, the library will omit all messages.
 \param      Log message callback function.
*/
VVENC_DECL void vvenc_registerMsgCbf( std::function<void( int, const char*, va_list )> msgCbf );

///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
VVENC_DECL std::string setSIMDExtension( const std::string& simdId );

VVENC_DECL bool  vvenc_isTracingEnabled();           // checks if library has tracing supported enabled (see ENABLE_TRACING).
VVENC_DECL const char* vvenc_getCompileInfoString(); // creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
VVENC_DECL void   vvenc_decodeBitstream( const char* FileName);

VVENC_DECL int  vvenc_getWidthOfComponent( const vvencChromaFormat& chFmt, const int frameWidth, const int compId );
VVENC_DECL int  vvenc_getHeightOfComponent( const vvencChromaFormat& chFmt, const int frameHeight, const int compId );

VVENC_NAMESPACE_END

#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /*_VVENC_H_*/
