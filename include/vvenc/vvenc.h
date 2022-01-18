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


/*
  The vvencEncoder struct provides the encoder's user interface.
  The simplest way to use the encoder is to call vvenc_encoder_create() to create an encoder instance and initialize it by using vvenc_encoder_open() with the
  the given vvenc_config. After initialization the encoding of the video is performed by using the vvenc_encode() method to hand over frame by frame in display order
  and retrieve the compressed bitstream chunks of already processed pictures. The encoding can be end by calling flush() that causes the encoder to finish encoding of all pending pictures.
  Finally calling vvenc_encoder_close() releases all allocated resources held by the encoder internally.
*/


/* vvencEncoder:
 * opaque handler for the decoder */
typedef struct vvencEncoder vvencEncoder;

/* vvdecLoggingCallback:
   callback function to receive messages of the encoder library
*/
typedef void (*vvencLoggingCallback)(void*, int, const char*, va_list);

/*
  \enum ErrorCodes
  The enum ErrorCodes enumerates error codes returned by the encoder.
*/
enum ErrorCodes
{
  VVENC_OK                   = 0,      // success
  VVENC_ERR_UNSPECIFIED      = -1,     // unspecified malfunction
  VVENC_ERR_INITIALIZE       = -2,     // encoder not initialized or tried to initialize multiple times
  VVENC_ERR_ALLOCATE         = -3,     // internal allocation error
  VVENC_NOT_ENOUGH_MEM       = -5,     // allocated memory to small to receive encoded data. After allocating sufficient memory the failed call can be repeated.
  VVENC_ERR_PARAMETER        = -7,     // inconsistent or invalid parameters
  VVENC_ERR_NOT_SUPPORTED    = -10,    // unsupported request
  VVENC_ERR_RESTART_REQUIRED = -11,    // encoder requires restart
  VVENC_ERR_CPU              = -30     // unsupported CPU SSE 4.1 needed
};

/*
  The struct vvencYUVPlane contains the data of an plane of an uncompressed input picture.
*/
typedef struct vvencYUVPlane
{
  int16_t*  ptr;                       // pointer to plane buffer
  int       width;                     // width of the plane
  int       height;                    // height of the plane
  int       stride;                    // stride (width + left margin + right margins) of plane in samples
}vvencYUVPlane;

/*
  The struct vvencYUVBuffer contains the data and attributes to hand over the uncompressed input picture and metadata related to picture.
*/
typedef struct vvencYUVBuffer
{
  vvencYUVPlane planes[ 3 ];           // plane buffer for 3 components (yuv)
  uint64_t      sequenceNumber;        // sequence number of the picture
  uint64_t      cts;                   // composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  bool          ctsValid;              // composition time stamp valid flag (true: valid, false: CTS not set)
}vvencYUVBuffer;

/* vvenc_YUVBuffer_alloc:
   Allocates an vvencYUVBuffer instance.
   The returned vvencYUVBuffer is set to default values.
   The payload memory must be allocated seperately by using vvenc_YUVBuffer_alloc_buffer.
   To free the memory use vvenc_YUVBuffer_free.
*/
VVENC_DECL vvencYUVBuffer* vvenc_YUVBuffer_alloc( void );

/* vvenc_YUVBuffer_free:
   release storage of an vvencYUVBuffer instance.
   The payload memory is also released if the flag freePicBuffer is set.
*/
VVENC_DECL void vvenc_YUVBuffer_free(vvencYUVBuffer *yuvBuffer, bool freePicBuffer );

/* vvenc_YUVBuffer_default:
  Initialize vvencYUVBuffer structure to default values
*/
VVENC_DECL void vvenc_YUVBuffer_default(vvencYUVBuffer *yuvBuffer );

/* vvenc_YUVBuffer_alloc_buffer:
   Allocates the payload buffer of a vvencYUVBuffer instance.
   To free the buffer memory use vvenc_YUVBuffer_free_buffer.
*/
VVENC_DECL void vvenc_YUVBuffer_alloc_buffer( vvencYUVBuffer *yuvBuffer, const vvencChromaFormat chFmt, const int frameWidth, const int frameHeight );

/* vvenc_YUVBuffer_free_buffer:
   release storage of the payload in a vvencYUVBuffer instance.
*/
VVENC_DECL void vvenc_YUVBuffer_free_buffer( vvencYUVBuffer *yuvBuffer );

// ----------------------------------------


/*
  The struct vvencAccessUnit contains attributes that are assigned to the compressed output of the encoder for a specific input picture.
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

  uint64_t        cts;                 // composition time stamp in TicksPerSecond (see vvenc_config)
  uint64_t        dts;                 // decoding time stamp in TicksPerSecond (see vvenc_config)
  bool            ctsValid;            // composition time stamp valid flag (true: valid, false: CTS not set)
  bool            dtsValid;            // decoding time stamp valid flag (true: valid, false: DTS not set)
  bool            rap;                 // random access point flag (true: AU is random access point, false: sequential access)
  vvencSliceType  sliceType;           // slice type (I/P/B) */
  bool            refPic;              // reference picture
  int             temporalLayer;       // temporal layer
  uint64_t        poc;                 // picture order count

  int             status;              // additional info (see Status)
  int             essentialBytes;      // number of bytes in nalus of type SLICE_*, DCI, VPS, SPS, PPS, PREFIX_APS, SUFFIX_APS
  char            infoString[VVENC_MAX_STRING_LEN]; // debug info from inside the encoder
} vvencAccessUnit;

/* vvenc_accessUnit_alloc:
   Allocates an vvencAccessUnit instance.
   The returned accessUnit is set to default values.
   The payload memory must be allocated seperately by using vvenc_accessUnit_alloc_payload.
   To free the memory use vvencAccessUnit_free.
*/
VVENC_DECL vvencAccessUnit* vvenc_accessUnit_alloc( void );

/* vvdec_accessUnit_free:
   release storage of an vvdecAccessUnit instance.
   The payload memory is also released if not done yet.
*/
VVENC_DECL void vvenc_accessUnit_free(vvencAccessUnit *accessUnit, bool freePayload );

/* vvenc_accessUnit_alloc_payload:
   Allocates the memory for an accessUnit payload.
   To free the memory use vvdecAccessUnit_free_payload.
   When the vvencAccessUnit memory is released the payload memory is also released.
*/
VVENC_DECL void vvenc_accessUnit_alloc_payload(vvencAccessUnit *accessUnit, int payload_size );

/* vvenc_accessUnit_free_payload:
   release storage of the payload in an vvencAccessUnit instance.
*/
VVENC_DECL void vvenc_accessUnit_free_payload(vvencAccessUnit *accessUnit );

/* vvenc_accessUnit_reset:
  resets vvdencAccessUnit structure to its default values. payload data will not be resetted.
*/
VVENC_DECL void vvenc_accessUnit_reset(vvencAccessUnit *accessUnit );

/* vvenc_accessUnit_default:
  Initialize vvencAccessUnit structure to default values (including au payload)
*/
VVENC_DECL void vvenc_accessUnit_default(vvencAccessUnit *accessUnit );

/*
 This method returns the encoder version number as a string.
 \param      None
 \retval     std::string returns the version number
*/
VVENC_DECL const char* vvenc_get_version( void );

/* vvenc_encoder_create
  This method creates a vvenc encoder instance.
  \param[in]  none.
  \retval     vvencEncoder pointer of the encoder handler if successful, otherwise NULL
  \pre        The encoder must not be initialized (pointer of decoder handler must be null).
*/
VVENC_DECL vvencEncoder* vvenc_encoder_create( void );

/* vvenc_encoder_open
  This method initializes the encoder instance.
  This method is used to initially set up the encoder with the assigned encoder parameter struct.
  The method fails if the encoder is already initialized or if the assigned parameter struct
  does not pass the consistency check. Other possibilities for an unsuccessful call are missing encoder license, or an machine with
  insufficient CPU-capabilities.
  \param[in]  vvencEncoder pointer to opaque handler.
  \param[in]  vvenc_config* pointer to vvenc_config struct that holds initial encoder parameters.
  \retval     int  if non-zero an error occurred (see ErrorCodes), otherwise the return value indicates success VVENC_OK
  \pre        The encoder must not be initialized.
*/
VVENC_DECL int vvenc_encoder_open( vvencEncoder*, vvenc_config* );

/* vvenc_encoder_close
 This method resets the encoder instance.
 This method clears the encoder and releases all internally allocated memory.
 Calling uninit cancels all pending encoding calls. In order to finish pending input pictures use the flush method.
 \param[in]  vvencEncoder pointer to opaque handler.
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        None
*/
VVENC_DECL int vvenc_encoder_close(vvencEncoder *);

/* vvencRecYUVBufferCallback:
   callback function to receive reconstructed yuv data of an encoded picture
*/
typedef void (*vvencRecYUVBufferCallback)(void*, vvencYUVBuffer* );

/* vvenc_encoder_set_RecYUVBufferCallback
 This method sets the callback to get the reconstructed YUV buffer.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[in]  ctx pointer of the caller, if not needed set it to null
 \param[in]  implementation of the callback
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        None
*/
VVENC_DECL int vvenc_encoder_set_RecYUVBufferCallback(vvencEncoder *, void * ctx, vvencRecYUVBufferCallback callback );

/* vvenc_init_pass
  This method initializes the encoder instance in dependency to the encoder pass.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[in]  pass number of current pass to init (0: first pass, 1: second pass )
 \param[in]  rate control statistics file name
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        None
*/
VVENC_DECL int vvenc_init_pass( vvencEncoder *, int pass, const char * statsFName );

/* vvenc_encode
  This method encodes a picture.
  Uncompressed input pictures are passed to the encoder in display order. A compressed bitstream chunks is returned by filling the assigned AccessUnit struct.
  Data in AcccessUnit struct are valid if the encoder call returns success and the UsedSize attribute is non-zero.
  If the input parameter pcInputPicture is NULL, the encoder just returns a pending bitstream chunk if available.
  If the call returns VVENC_NOT_ENOUGH_MEM, the BufSize attribute in AccessUnit struct indicates that the buffer is to small to retrieve the compressed data waiting for delivery.
  In this case the UsedSize attribute returns the minimum buffersize required to fetch the pending chunk. After allocating sufficient memory the encoder can retry the last call with the parameter pcInputPicture set to NULL to prevent encoding the last picture twice.
  \param[in]  vvencEncoder pointer to opaque handler
  \param[in]  pcYUVBuffer pointer to vvencYUVBuffer structure containing uncompressed picture data and meta information, to flush the encoder YUVBuffer must be NULL.
  \param[out] accessUnit pointer to vvencAccessUnit that retrieves compressed access units and side information, data are valid if UsedSize attribute is non-zero and the call was successful.
  \param[out] encodeDone pointer to flag that indicates that the encoder completed the last frame after flushing.
  \retval     int if non-zero an error occurred, otherwise the retval indicates success VVENC_OK
  \pre        The encoder has to be initialized successfully.
*/
VVENC_DECL int vvenc_encode( vvencEncoder *, vvencYUVBuffer* YUVBuffer, vvencAccessUnit* accessUnit, bool* encodeDone );

/* vvenc_get_config
 This method fetches the current encoder configuration.
 The method fails if the encoder is not initialized.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[in]  vvenc_config reference to an vvenc_config struct that returns the current encoder setup.
 \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
 \pre        The encoder has to be initialized.
*/
VVENC_DECL int vvenc_get_config( vvencEncoder *,vvenc_config * );

/* vvenc_reconfig
 This method reconfigures the encoder instance.
 This method is used to change encoder settings during the encoding process when the encoder was already initialized.
 Some parameter changes might require an internal encoder restart, especially when previously used parameter sets VPS, SPS or PPS
 become invalid after the parameter change. If changes are limited to TargetBitRate or QP changes then the encoder continues encoding
 without interruption, using the new parameters. Some parameters e.g. NumTheads are not reconfigurable - in this case the encoder returns an Error.
 The method fails if the encoder is not initialized or if the assigned parameter set given in vvenc_config struct
 does not pass the consistency and parameter check.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[in]  vvenc_config const reference to vvenc_config struct that holds the new encoder parameters.
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        The encoder has to be initialized successfully.
*/
VVENC_DECL int vvenc_reconfig( vvencEncoder *, const vvenc_config * );

/* vvenc_check_config
 This method checks the passed configuration.
 The method fails if the encoder is not initialized.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[in]  rcVVCEncParameter reference to an VVCEncParameter struct that returns the current encoder setup.
 \retval     int VVENC_ERR_PARAMETER indicates a parameter error, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int vvenc_check_config( vvencEncoder *, const vvenc_config * );

/* vvenc_get_headers
 This method returns the headers (SPS,PPS,...) that are used.
 All init calls (vvenc_encoder_open, vvenc_init_pass) must be called in advance.
 \param[in]  vvencEncoder pointer to opaque handler
 \param[out] accessUnit pointer to vvencAccessUnit that retrieves compressed access units containing all headers.    
 \retval     int negative indicates an error, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int vvenc_get_headers(vvencEncoder *, vvencAccessUnit * );

/* vvenc_get_last_error
 This method returns the last occurred error as a string.
 \param[in]  vvencEncoder pointer to opaque handler
 \retval     const char empty string for no error assigned
*/
VVENC_DECL const char* vvenc_get_last_error( vvencEncoder * );

/* vvenc_get_enc_information
 This method returns information about the encoder as a string.
 \param[in]  vvencEncoder pointer to opaque handler
 \retval     const char* encoder information
*/
VVENC_DECL const char* vvenc_get_enc_information( vvencEncoder * );

/* vvenc_get_num_lead_frames
 This method the number of needed lead frames (used for MCTF)
 \param[in]  vvencEncoder pointer to opaque handler
 \retval     number of leading frames
*/
VVENC_DECL int vvenc_get_num_lead_frames( vvencEncoder * );

/* vvenc_get_num_trail_frames
 This method the number of needed trailing frames (used for MCTF)
 \param[in]  vvencEncoder pointer to opaque handler
 \retval     number of trailing frames
*/
VVENC_DECL int vvenc_get_num_trail_frames( vvencEncoder * );

/* vvenc_print_summary
 This method prints the summary of a encoder run.
 \param[in]  vvencEncoder pointer to opaque handler
 \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int vvenc_print_summary( vvencEncoder * );

/* vvenc_get_error_msg
 This static function returns a string according to the passed parameter nRet.
 \param[in]  nRet return value code to translate
 \retval[ ]  const char*  empty string for no error
*/
VVENC_DECL const char* vvenc_get_error_msg( int nRet );

/* vvenc_set_logging_callback *deprecated*
 This method registers a global log message callback function to the encoder library.
 If no such function has been registered, the library will omit all messages.
 *deprecated* - This method is deprecated since it uses a global logger and will be removed in the next major version.
                Please use the method vvenc_set_logging_callback(vvenc_config,void *,vvencLoggingCallback) to register a thread safe local looger
 \param[in]  ctx pointer of the caller, if not needed set it to null
 \paramin]   Log message callback function.
 \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int vvenc_set_logging_callback( void * ctx, vvencLoggingCallback callback );

/* vvenc_get_compile_info_string
 creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
 \retval[ ]  const char* compiler infoa as string
*/
VVENC_DECL const char* vvenc_get_compile_info_string( void );

/* vvenc_set_SIMD_extension
  tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
 \param      const char* simdId: empty string to set highest possible extension, otherwise set simd extension
 \retval[ ]  const char* current simd exentsion
*/
VVENC_DECL const char* vvenc_set_SIMD_extension( const char* simdId );

/* vvenc_get_height_of_component

 \param      chFmt  Chroma Format
 \param      frameWidth width
 \param      compId component ID
 \retval[ ]  width of compontent
*/
VVENC_DECL int  vvenc_get_width_of_component( const vvencChromaFormat chFmt, const int frameWidth, const int compId );

/* vvenc_get_height_of_component
 \param      chFmt Chroma Format
 \param      frameHeight
 \param      compId component ID
 \retval[ ]  height of compontent
*/
VVENC_DECL int  vvenc_get_height_of_component( const vvencChromaFormat chFmt, const int frameHeight, const int compId );

/* Debug section */

/* vvenc_is_tracing_enabled
 checks if library has tracing supported enabled (see ENABLE_TRACING).
 \retval[ ]  true if tracing is enabled, else false
*/
VVENC_DECL bool  vvenc_is_tracing_enabled( void );

/* vvenc_decode_bitstream
 \param[in]  FileName of bitstream that should be decoded
 \param[in]  trcFile filename of a trace rule file
 \param[in]  trcRule trace rules
 \retval     int VVENC_ERR_INITIALIZE indicates the encoder was not successfully initialized in advance, otherwise the return value VVENC_OK indicates success.
*/
VVENC_DECL int   vvenc_decode_bitstream( const char* FileName, const char* trcFile, const char* trcRule);


VVENC_NAMESPACE_END

#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /*_VVENC_H_*/
