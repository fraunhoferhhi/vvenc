/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur F�rderung der angewandten Forschung e.V. & The VVenC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

#include "SEIFilmGrainAnalyzer.h"

#include "CommonLib/MCTF.h"
#include "TrQuant_EMT.h"

using namespace vvenc;

// POLYFIT
static constexpr int      MAXORDER = 8;                                   // maximum order of polynomial fitting
static constexpr int      MAX_REAL_SCALE = 16;
static constexpr int      ORDER = 4;                                      // order of polynomial function
static constexpr int      QUANT_LEVELS = 4;                               // number of quantization levels in lloyd max quantization

static constexpr int      MIN_ELEMENT_NUMBER_PER_INTENSITY_INTERVAL = 8;
static constexpr int      MIN_POINTS_FOR_INTENSITY_ESTIMATION = 40;       // 5*8 = 40; 5 intervals with at least 8 points
static constexpr int      MIN_BLOCKS_FOR_CUTOFF_ESTIMATION = 2;           // 2 blocks of 64 x 64 size
static constexpr int      POINT_STEP = 16;                                // step size in point extension
static constexpr int      MAX_NUM_POINT_TO_EXTEND = 4;                    // max point in extension
static constexpr double   POINT_SCALE = 1.25;                             // scaling in point extension
static constexpr double   VAR_SCALE_DOWN = 1.2;                           // filter out large points
static constexpr double   VAR_SCALE_UP = 0.6;                             // filter out large points
static constexpr int      NUM_PASSES = 2;                                 // number of passes when fitting the function
static constexpr int      NBRS = 1;                                       // minimum number of surrounding points in order to keep it for further analysis (within the widnow range)
static constexpr int      WINDOW = 1;                                     // window to check surrounding points
static constexpr int      MIN_INTENSITY = 40;
static constexpr int      MAX_INTENSITY = 950;

static constexpr int      MAX_ALLOWED_MODEL_VALUES = 3;
static constexpr int      MAX_NUM_MODEL_VALUES = 6;                       // Maximum number of model values supported in FGC SEI

static constexpr int      BLK_8 = 8;
static constexpr int      BLK_16 = 16;
static constexpr int      BLK_32 = 32;
static constexpr int      BIT_DEPTH_8 = 8;


static constexpr int      MAX_BLOCKS = 40000;                             // higher than (3840*2160)/(16*16)

const int m_gx[CONV_HEIGHT_S][CONV_WIDTH_S]{ { -1, 0, 1 }, { -2, 0, 2 }, { -1, 0, 1 } };
const int m_gy[CONV_HEIGHT_S][CONV_WIDTH_S]{ { -1, -2, -1 }, { 0, 0, 0 }, { 1, 2, 1 } };

constexpr double FGAnalyzer::m_tapFilter[3];

void gradient_core ( PelStorage *buff1,
                     PelStorage *buff2,
                     PelStorage *tmpBuf1,
                     PelStorage *tmpBuf2,
                     uint32_t width,
                     uint32_t height,
                     uint32_t bitDepth,
                     ComponentID compID )
{
  // buff1 - magnitude; buff2 - orientation (Only luma in buff2)
  const uint32_t convWidthS = CONV_WIDTH_S;
  const uint32_t convHeightS = CONV_HEIGHT_S;
  const int maxClpRange = (1 << bitDepth) - 1;
  const int padding     = convWidthS / 2;

  buff1->get(compID).extendBorderPel( padding,
                                      padding );

  // Gx
  for (int i = 0; i < width; i++)
  {
    for (int j = 0; j < height; j++)
    {
      int acc = 0;
      for (int x = 0; x < convWidthS; x++)
      {
        for (int y = 0; y < convHeightS; y++)
        {
          acc += (buff1->get(compID).at(x - convWidthS / 2 + i, y - convHeightS / 2 + j) * m_gx[x][y]);
        }
      }
      tmpBuf1->Y().at(i, j) = acc;
    }
  }

  // Gy
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      int acc = 0;
      for ( int x = 0; x < convWidthS; x++ )
      {
        for ( int y = 0; y < convHeightS; y++ )
        {
          acc += (buff1->get(compID).at(x - convWidthS / 2 + i, y - convHeightS / 2 + j) * m_gy[x][y]);
        }
      }
      tmpBuf2->Y().at(i, j) = acc;
    }
  }

  // magnitude
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      Pel tmp                     = static_cast<Pel>((abs(tmpBuf1->Y().at(i, j)) + abs(tmpBuf2->Y().at(i, j))) / 2);
      buff1->get(compID).at(i, j) = static_cast<Pel>( Clip3((Pel) 0, (Pel) maxClpRange, tmp) );
    }
  }

  // Loop through each pixel
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      // Calculate edge direction angle
      Pel Dx = tmpBuf1->Y().at( i, j );
      Pel Dy = tmpBuf2->Y().at( i, j );
      float theta = 0.0;
      int quantized_direction = 0;

      if ( Dx == 0 )
      {
        if ( Dy == 0 )
          quantized_direction = 0;
        else
          quantized_direction = 90;
      }
      else
      {
        theta= ( atan(static_cast<double>( Dy )/(double)static_cast<double>( Dx )) ) ;
        if ( Dx < 0 )
        {
          if ( Dy >= 0 )
            theta += static_cast<float>( PI );
          else
            theta -= static_cast<float>( PI );
        }
        theta=fabs( theta );
        /* Convert actual edge direction to approximate value - quantize directions */
        if (( theta <= pi_8 ) || ( pi_7_8 < theta ))
        {
          quantized_direction = 0;
        }
        if (( pi_8 < theta ) && ( theta <= pi_3_8 ))
        {
          if ( Dy > 0 )
            quantized_direction = 45;
          else
            quantized_direction = 135;
        }
        if (( pi_3_8 < theta ) && ( theta <= pi_5_8 ))
        {
          quantized_direction = 90;
        }
        if (( pi_5_8 < theta ) && ( theta <= pi_7_8 ))
        {
          if ( Dy > 0 )
            quantized_direction = 135;
          else
            quantized_direction = 45;
        }
      }
      buff2->get(ComponentID(0)).at( i, j ) = quantized_direction;
    }
  }
  buff1->get(compID).extendBorderPel( padding, 
                                      padding );   // extend border for the next steps
}

// ====================================================================================================================
// Edge detection - Canny
// ====================================================================================================================

Canny::Canny()
{
  // init();
  gradient=gradient_core;
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  initFGACannyX86();
#endif
}

Canny::~Canny()
{
  // uninit();
}

void Canny::init ( uint32_t width,
                   uint32_t height,
                   ChromaFormat inputChroma )
{
  if (!m_orientationBuf)
  {
    m_orientationBuf = new PelStorage;
    m_orientationBuf->create( inputChroma,
                              Area(0, 0, width, height) );
  }

  if ( !m_gradientBufX )
  {
    m_gradientBufX = new PelStorage;
    m_gradientBufX->create ( inputChroma,
                             Area(0, 0, width, height) );
  }

  if ( !m_gradientBufY )
  {
    m_gradientBufY = new PelStorage;
    m_gradientBufY->create ( inputChroma,
                             Area(0, 0, width, height) );
  }
}

void Canny::destroy()
{
  if ( m_orientationBuf )
  {
    m_orientationBuf->destroy();
    delete m_orientationBuf;
    m_orientationBuf = nullptr;
  }

  if ( m_gradientBufX )
  {
    m_gradientBufX->destroy();
    delete m_gradientBufX;
    m_gradientBufX = nullptr;
  }

  if ( m_gradientBufY )
  {
    m_gradientBufY->destroy();
    delete m_gradientBufY;
    m_gradientBufY = nullptr;
  }
}

void Canny::suppressNonMax ( PelStorage *buff1,
                             PelStorage *buff2,
                             uint32_t width,
                             uint32_t height,
                             ComponentID compID )
{
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      int rowShift = 0, colShift = 0;
      switch ( buff2->get( ComponentID(0) ).at( i, j ) )
      {
      case 0:
        rowShift = 1;
        colShift = 0;
        break;
      case 45:
        rowShift = 1;
        colShift = 1;
        break;
      case 90:
        rowShift = 0;
        colShift = 1;
        break;
      case 135:
        rowShift = -1;
        colShift = 1;
        break;
      default: THROW("Unsupported gradient direction."); break;
      }

      Pel pelCurrent             = buff1->get(compID).at( i, j );
      Pel pelEdgeDirectionTop    = buff1->get(compID).at( i + rowShift, j + colShift );
      Pel pelEdgeDirectionBottom = buff1->get(compID).at( i - rowShift, j - colShift );
      if (( pelCurrent < pelEdgeDirectionTop ) || ( pelCurrent < pelEdgeDirectionBottom ))
      {
        buff2->get(ComponentID(0)).at( i, j ) = 0;   // supress
      }
      else
      {
        buff2->get(ComponentID(0)).at( i, j ) = buff1->get(compID).at( i, j );   // keep
      }
    }
  }
  buff1->get(compID).copyFrom( buff2->get( ComponentID(0) ) );
}

void Canny::doubleThreshold ( PelStorage *buff,
                              uint32_t width,
                              uint32_t height,
                              uint32_t bitDepth,
                              ComponentID compID )
{
  Pel strongPel = ( static_cast<Pel>( 1 ) << bitDepth) - 1;
  Pel weekPel   = ( static_cast<Pel>( 1 ) << (bitDepth - 1)) - 1;

  Pel highThreshold = 0;
  Pel lowThreshold  = strongPel;
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      highThreshold = std::max<Pel>( highThreshold,
                                     buff->get(compID).at( i, j ) );
    }
  }

  // global low and high threshold
  lowThreshold = static_cast<Pel>( m_lowThresholdRatio * highThreshold );
  highThreshold = Clip3( 0,
                         (1 << bitDepth) - 1,
                         m_highThresholdRatio * lowThreshold);   // Canny recommended a upper:lower ratio between 2:1 and 3:1.

  // strong, week, supressed
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      if ( buff->get(compID).at( i, j ) > highThreshold )
      {
        buff->get(compID).at( i, j ) = strongPel;
      }
      else if ( buff->get(compID).at( i, j ) <= highThreshold && buff->get(compID).at( i, j ) > lowThreshold )
      {
        buff->get(compID).at( i, j ) = weekPel;
      }
      else
      {
        buff->get(compID).at( i, j ) = 0;
      }
    }
  }

  buff->get(compID).extendBorderPel ( 1, 1 );   // extend one pixel on each side for the next step
}

void Canny::edgeTracking ( PelStorage *buff,
                           uint32_t width,
                           uint32_t height,
                           uint32_t windowWidth,
                           uint32_t windowHeight,
                           uint32_t bitDepth,
                           ComponentID compID )
{
  Pel strongPel = (static_cast<Pel>(1) << bitDepth) - 1;
  Pel weakPel   = (static_cast<Pel>(1) << (bitDepth - 1)) - 1;

  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      if ( buff->get(compID).at( i, j ) == weakPel )
      {
        bool strong = false;

        for ( int x = 0; x < windowWidth; x++ )
        {
          for ( int y = 0; y < windowHeight; y++ )
          {
            if ( buff->get(compID).at( x - windowWidth / 2 + i, y - windowHeight / 2 + j ) == strongPel )
            {
              strong = true;
              break;
            }
          }
        }

        if ( strong )
        {
          buff->get(compID).at( i, j ) = strongPel;
        }
        else
        {
          buff->get(compID).at( i, j ) = 0;   // supress
        }
      }
    }
  }
}

void Canny::detect_edges ( const PelStorage *orig,
                           PelStorage *dest,
                           uint32_t uiBitDepth,
                           ComponentID compID )
{
  /* No noise reduction - Gaussian blur is skipped;
   Gradient calculation;
   Non-maximum suppression;
   Double threshold;
   Edge Tracking by Hysteresis.*/

  uint32_t width      = orig->get( compID ).width,
           height     = orig->get( compID ).height;       // Width and Height of current frame
  uint32_t convWidthS  = CONV_WIDTH_S,
           convHeightS = CONV_HEIGHT_S;                 // Pixel's row and col positions for Sobel filtering
  uint32_t bitDepth    = uiBitDepth;

  dest->get(compID).copyFrom( orig->getBuf( compID ) );   // we skip blur in canny detector to catch as much as possible edges and textures

  /* Gradient calculation */
  gradient ( dest,
             m_orientationBuf,
             m_gradientBufX,
             m_gradientBufY,
             width,
             height,
             bitDepth,
             compID );

  /* Non - maximum suppression */
  suppressNonMax ( dest,
                   m_orientationBuf,
                   width,
                   height,
                   compID );

  /* Double threshold */
  doubleThreshold ( dest,
                    width,
                    height, 
                    bitDepth,
                    compID );

  /* Edge Tracking by Hysteresis */
  edgeTracking ( dest,
                 width,
                 height,
                 convWidthS,
                 convHeightS,
                 bitDepth,
                 compID ); 
}

// ====================================================================================================================
// Morphologigal operations - Dilation and Erosion
// ====================================================================================================================
int dilation_core ( PelStorage *buff,
                    PelStorage *Wbuf,
                    uint32_t bitDepth,
                    ComponentID compID,
                    int numIter,
                    int iter,
                    Pel Value )
{
  if ( iter == numIter )
  {
    return iter;
  }
  uint32_t width      = buff->get( compID ).width;
  uint32_t height     = buff->get( compID ).height;   // Width and Height of current frame
  uint32_t windowSize = KERNELSIZE;
  uint32_t padding    = windowSize / 2;

  Wbuf->bufs[0].copyFrom( buff->get( compID ) );

  buff->get(compID).extendBorderPel( padding,
                                     padding );

  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      bool strong = false;
      for ( int x = 0; x < windowSize; x++ )
      {
        for ( int y = 0; y < windowSize; y++ )
        {
          if ( buff->get( compID ).at( x - windowSize / 2 + i, y - windowSize / 2 + j ) == Value )
          {
            strong = true;
            break;
          }
        }
        if ( strong ) break;
      }
      if ( strong )
      {
        Wbuf->get(ComponentID(0)).at( i, j ) = Value;
      }
    }
  }

  buff->get(compID).copyFrom( Wbuf->bufs[0] );

  return dilation_core ( buff,
                         Wbuf,
                         bitDepth,
                         compID,
                         numIter,
                         ++iter,
                         Value );

  return iter;
}

Morph::Morph()
{
  // init();
  dilation=dilation_core;
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  initFGAMorphX86();
#endif
}

Morph::~Morph()
{
  // uninit();
}

void Morph::init ( uint32_t width,
                   uint32_t height )
{
  if ( !m_dilationBuf )
  {
    m_dilationBuf = new PelStorage;
    m_dilationBuf->create ( VVENC_CHROMA_400,
                            Area( 0, 0, width, height ) );
  }
  if ( !m_dilationBuf2 )
  {
    m_dilationBuf2 = new PelStorage;
    m_dilationBuf2->create ( VVENC_CHROMA_400,
                             Area( 0, 0, width >> 1, height >> 1 ) );
  }
  if ( !m_dilationBuf4 )
  {
    m_dilationBuf4 = new PelStorage;
    m_dilationBuf4->create( VVENC_CHROMA_400,
                              Area( 0, 0, width >> 2, height >> 2 ) );
  }
}

void Morph::destroy ()
{
  if ( m_dilationBuf )
  {
    m_dilationBuf->destroy();
    delete m_dilationBuf;
    m_dilationBuf = nullptr;
  }
  if ( m_dilationBuf2 )
  {
    m_dilationBuf2->destroy();
    delete m_dilationBuf2;
    m_dilationBuf2 = nullptr;
  }
  if ( m_dilationBuf4 )
  {
    m_dilationBuf4->destroy();
    delete m_dilationBuf4;
    m_dilationBuf4 = nullptr;
  }
}

int calcMeanCore ( const Pel* org,
                   const ptrdiff_t origStride,
                   const int w,
                   const int h )
{
  // calculate average
  int avg = 0;
  for( int y1 = 0; y1 < h; y1++ )
  {
    for( int x1 = 0; x1 < w; x1++ )
    {
      avg = avg + *( org + x1 + y1 * origStride );
    }
  }
  return avg;
}

// ====================================================================================================================
// Film Grain Analysis Functions
// ====================================================================================================================
FGAnalyzer::FGAnalyzer()
{
}

FGAnalyzer::~FGAnalyzer()
{
}

// initialize film grain parameters
void FGAnalyzer::init ( const int width,
                        const int height,
                        const ChromaFormat inputChroma,
                        const int *outputBitDepths,
                        const bool doAnalysis[] )
{
  m_log2ScaleFactor = 2;
  for (int i = 0; i < ComponentID::MAX_NUM_COMP; i++)
  {
    m_compModel[i].presentFlag           = true;
    m_compModel[i].numModelValues        = 3;
    m_compModel[i].numIntensityIntervals = 1;
    m_compModel[i].intensityValues.resize(VVENC_MAX_NUM_INTENSITIES);
    for ( int j = 0; j < VVENC_MAX_NUM_INTENSITIES; j++ )
    {
      m_compModel[i].intensityValues[j].intensityIntervalLowerBound = 10;
      m_compModel[i].intensityValues[j].intensityIntervalUpperBound = 250;
      m_compModel[i].intensityValues[j].compModelValue.resize( MAX_ALLOWED_MODEL_VALUES );
      for ( int k = 0; k < m_compModel[i].numModelValues; k++ )
      {
        // half intensity for chroma. Provided value is default value, manually tuned.
        m_compModel[i].intensityValues[j].compModelValue[k] = i == 0 ? 26 : 13;
      }
    }
    m_doAnalysis[i] = doAnalysis[i];
  }

  // initialize picture parameters and create buffers
  m_bitDepths                   = const_cast<int*>( outputBitDepths );
  m_inputChromaFormat           = inputChroma;
  // Allocate memory for m_coeffBuf and m_dctGrainBlockList
  m_coeffBuf = (TCoeff*)xMalloc( TCoeff, width * height );
  int N = (width * height) / (DATA_BASE_SIZE * DATA_BASE_SIZE);
  m_dctGrainBlockList = new CoeffBuf[N];

  std::fill( std::begin(vecMean), std::end(vecMean), 0 );
  std::fill( std::begin(vecVar), std::end(vecVar), 0 );

  // Connect portions of m_coeffBuf memory with m_dctGrainBlockList
  for ( int i = 0; i < N; ++i )
  {
    m_dctGrainBlockList[i].buf = m_coeffBuf + i * ( DATA_BASE_SIZE * DATA_BASE_SIZE );
    m_dctGrainBlockList[i].stride = DATA_BASE_SIZE;
    m_dctGrainBlockList[i].height = m_dctGrainBlockList[i].width = DATA_BASE_SIZE;
  }

  m_edgeDetector.init ( width,
                        height,
                        inputChroma );

  m_morphOperation.init ( width,
                          height );

  int margin = m_edgeDetector.m_convWidthG / 2;   // set margin for padding for filtering
  int      newWidth2 = width / 2;
  int      newHeight2 = height / 2;
  int      newWidth4 = width / 4;
  int      newHeight4 = height / 4;

  if ( !m_maskBuf )
  {
    m_maskBuf = new PelStorage;
    m_maskBuf->create ( inputChroma,
                        Area(0, 0, width, height),
                        0, margin,
                        0, false );
  }

  if ( !m_grainEstimateBuf )
  {
     m_grainEstimateBuf = new PelStorage;
     m_grainEstimateBuf->create( inputChroma,
                                Area(0, 0, width, height),
                                0, 0,
                                0, false );
  }

  if ( !m_workingBufSubsampled2 )
  {
    m_workingBufSubsampled2 = new PelStorage;
    m_workingBufSubsampled2->create( inputChroma,
                                     Area(0, 0, newWidth2, newHeight2),
                                     0, margin,
                                     0, false );
  }

  if ( !m_maskSubsampled2 )
  {
    m_maskSubsampled2 = new PelStorage;
    m_maskSubsampled2->create( inputChroma,
                               Area(0, 0, newWidth2, newHeight2),
                               0, margin,
                               0, false );
  }
  if ( !m_workingBufSubsampled4 )
  {
    m_workingBufSubsampled4 = new PelStorage;
    m_workingBufSubsampled4->create( inputChroma,
                                     Area(0, 0, newWidth4, newHeight4),
                                     0, margin,
                                     0, false );
  }

  if ( !m_maskSubsampled4 )
  {
    m_maskSubsampled4 = new PelStorage;
    m_maskSubsampled4->create( inputChroma,
                               Area(0, 0, newWidth4, newHeight4),
                               0, margin,
                               0, false );
  }
  if ( !m_maskUpsampled )
  {
    m_maskUpsampled = new PelStorage;
    m_maskUpsampled->create( inputChroma,
                             Area(0, 0, width, height),
                             0, margin,
                             0, false );
  }
  if ( !m_DCTinout )
  {
    m_DCTinout = ( TCoeff* ) xMalloc( TCoeff, DATA_BASE_SIZE * DATA_BASE_SIZE );
  }
  if ( !m_DCTtemp )
  {
    m_DCTtemp = ( TCoeff* ) xMalloc( TCoeff, DATA_BASE_SIZE * DATA_BASE_SIZE );
  }

  calcVar=calcVarCore;
  calcMean=calcMeanCore;
  fastDCT2_64 = fastForwardDCT2_B64;

#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  initFGAnalyzerX86();
#endif

}

// delete picture buffers
void FGAnalyzer::destroy()
{
  if ( m_maskBuf != nullptr )
  {
    m_maskBuf->destroy();
    delete m_maskBuf;
    m_maskBuf = nullptr;
  }

  if ( m_grainEstimateBuf )
  {
    m_grainEstimateBuf->destroy();
    delete m_grainEstimateBuf;
    m_grainEstimateBuf = nullptr;
  }

  if ( m_workingBufSubsampled2 )
  {
    m_workingBufSubsampled2->destroy();
    delete m_workingBufSubsampled2;
    m_workingBufSubsampled2 = nullptr;
  }
  if ( m_maskSubsampled2 )
  {
    m_maskSubsampled2->destroy();
    delete m_maskSubsampled2;
    m_maskSubsampled2 = nullptr;
  }
  
  if ( m_workingBufSubsampled4 )
  {
    m_workingBufSubsampled4->destroy();
    delete m_workingBufSubsampled4;
    m_workingBufSubsampled4 = nullptr;
  }
  if ( m_maskSubsampled4 )
  {
    m_maskSubsampled4->destroy();
    delete m_maskSubsampled4;
    m_maskSubsampled4 = nullptr;
  }
  if ( m_maskUpsampled )
  {
    m_maskUpsampled->destroy();
    delete m_maskUpsampled;
    m_maskUpsampled = nullptr;
  }
  if ( m_DCTinout )
  {
    xFree( m_DCTinout );
    m_DCTinout = nullptr;
  }
  if ( m_DCTtemp )
  {
    xFree( m_DCTtemp );
    m_DCTtemp = nullptr;
  }

  xFree ( m_coeffBuf );

  if ( m_dctGrainBlockList )
  {
    delete[] m_dctGrainBlockList;
    m_dctGrainBlockList = nullptr;
  }

  // Clear vectors to release memory
  finalIntervalsandScalingFactors.clear();
  vec_mean_intensity.clear();
  vec_variance_intensity.clear();
  element_number_per_interval.clear();
  vecMean.clear();
  vecVar.clear();
  tmp_data_x.clear();
  tmp_data_y.clear();
  scalingVec.clear();
  quantVec.clear();
  coeffs.clear();

  m_edgeDetector.destroy ();
  m_morphOperation.destroy ();
}

// find flat and low complexity regions of the frame
void FGAnalyzer::findMask( ComponentID compId )
{
  const unsigned padding    = m_edgeDetector.m_convWidthG / 2;   // for filtering
  int bitDepth  = m_bitDepths[toChannelType( compId )];

  // Step 1: Subsample the original picture to two lower resolutions.
  subsample ( *m_workingBufSubsampled2,
              2,
              padding,
              compId );
  subsample ( *m_workingBufSubsampled4,
              4,
              padding,
              compId );

  /* Step 2: Full Resolution processing:
   * For each component(luma and chroma), detect edges and suppress low intensity regions.
   * Apply dilation to each component.*/
  m_edgeDetector.detect_edges ( m_workingBuf,
                                m_maskBuf,
                                bitDepth,
                                compId );
  suppressLowIntensity ( *m_workingBuf,
                         *m_maskBuf,
                         bitDepth,
                         compId );
  
  Pel strongPel = ( static_cast<Pel>( 1 ) << bitDepth ) - 1;
  m_morphOperation.dilation ( m_maskBuf,
                              m_morphOperation.m_dilationBuf,
                              bitDepth,
                              compId,
                              4,
                              0,
                              strongPel );
  
  
  /* Step 3: Subsampled 2 processing:
   * Detect edges and suppresses low intensity regions for each component.
   * Apply dilation to each component.
   * Upsample the result and combine it with the full-resolution mask.*/
  m_edgeDetector.detect_edges ( m_workingBufSubsampled2,
                                m_maskSubsampled2,
                                bitDepth,
                                compId );
  suppressLowIntensity ( *m_workingBufSubsampled2,
                         *m_maskSubsampled2,
                         bitDepth,
                         compId );
  
    
  m_morphOperation.dilation ( m_maskSubsampled2,
                              m_morphOperation.m_dilationBuf2,
                              bitDepth,
                              compId,
                              3,
                              0,
                              strongPel );


  // upsample, combine maskBuf and maskUpsampled
  upsample ( *m_maskSubsampled2,
             2,
             compId );
  combineMasks ( compId );

  /* Step 4: Subsampled 4 processing:
   * Detect edges and suppresses low intensity regions for each component.
   * Apply dilation to each component.
   * Upsample the result and combine it with the full-resolution mask.*/
  m_edgeDetector.detect_edges ( m_workingBufSubsampled4,
                                m_maskSubsampled4,
                                bitDepth,
                                compId );
  suppressLowIntensity ( *m_workingBufSubsampled4,
                         *m_maskSubsampled4,
                         bitDepth,
                         compId );

  m_morphOperation.dilation ( m_maskSubsampled4,
                              m_morphOperation.m_dilationBuf4,
                              bitDepth,
                              compId,
                              2,
                              0,
                              strongPel );

  // upsample, combine maskBuf and maskUpsampled
  upsample ( *m_maskSubsampled4,
             4,
             compId );
  combineMasks ( compId );

  /* Step 5: Final dilation and erosion
   * Apply final dilation to fill the holes and erosion for each component. */
  m_morphOperation.dilation ( m_maskBuf,
                              m_morphOperation.m_dilationBuf,
                              bitDepth,
                              compId,
                              2,
                              0,
                              strongPel );
  // erosion -> dilation with value 0
  m_morphOperation.dilation ( m_maskBuf,
                              m_morphOperation.m_dilationBuf,
                              bitDepth,
                              compId,
                              1,
                              0,
                              0 );
}

void FGAnalyzer::suppressLowIntensity ( const PelStorage &buff1,
                                        PelStorage &buff2,
                                        uint32_t bitDepth, 
                                        ComponentID compId )
{
  // buff1 - intensity values ( luma or chroma samples); buff2 - mask

  int width                 = buff2.get( compId ).width;
  int height                = buff2.get( compId ).height;
  Pel maxIntensity          = static_cast <Pel>( 1 << bitDepth ) - 1;
  Pel lowIntensityThreshold = static_cast<Pel>( m_lowIntensityRatio * maxIntensity );

  // strong, weak, supressed
  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      // Check if the intensity is below the threshold
      if ( buff1.get( compId ).at( i, j ) < lowIntensityThreshold )
      {
        // Set the corresponding mask value to maxIntensity
        buff2.get( compId ).at( i, j ) = maxIntensity;
      }
    }
  }
}

void FGAnalyzer::subsample ( PelStorage &output,
                             const int factor,
                             const int padding,
                             ComponentID compId ) const
{
  const int newWidth  = m_workingBuf->get( compId ).width / factor;
  const int newHeight = m_workingBuf->get( compId ).height / factor;

  const Pel *srcRow    = m_workingBuf->get( compId ).buf;
  const ptrdiff_t srcStride = m_workingBuf->get( compId ).stride;
  Pel *dstRow    = output.get( compId ).buf;   // output is tmp buffer with only one component for binary mask
  const ptrdiff_t dstStride = output.get( compId ).stride;

  for ( int y = 0; y < newHeight; y++, srcRow += factor * srcStride, dstRow += dstStride )
  {
    const Pel *inRow      = srcRow;
    const Pel *inRowBelow = srcRow + srcStride;
    Pel *      target     = dstRow;

    for ( int x = 0; x < newWidth; x++ )
    {
      target[x] = ( inRow[0] + inRowBelow[0] + inRow[1] + inRowBelow[1] + 2 ) >> 2;
      inRow += factor;
      inRowBelow += factor;
    }
  }

  if ( padding )
  {
    // Extend border with padding
    output.get( compId ).extendBorderPel ( padding,
                                           padding );
  }
}

void FGAnalyzer::upsample ( const PelStorage &input,
                            const int factor,
                            const int padding,
                            ComponentID compId ) const
{
  // binary mask upsampling
  // use simple replication of pixels

  const int width  = input.get(compId).width;
  const int height = input.get(compId).height;

  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
      Pel currentPel = input.get( compId ).at( i, j );

      for ( int x = 0; x < factor; x++ )
      {
        for ( int y = 0; y < factor; y++ )
        {
          m_maskUpsampled->get( compId ).at( i * factor + x, j * factor + y ) = currentPel;
        }
      }
    }
  }

  if ( padding )
  {
    m_maskUpsampled->get( compId ).extendBorderPel( padding,
                                                    padding );
  }
}

void FGAnalyzer::combineMasks( ComponentID compId )
{
  const int width = m_maskBuf->get( compId ).width;
  const int height = m_maskBuf->get( compId ).height;

  for ( int i = 0; i < width; i++ )
  {
    for ( int j = 0; j < height; j++ )
    {
        m_maskBuf->get( compId ).at( i, j ) = ( m_maskBuf->get( compId ).at( i, j ) | m_maskUpsampled->get( compId ).at( i, j ) );
    }
  }
}

// estimate cut-off frequencies and scaling factors for different intensity intervals
void FGAnalyzer::estimateGrainParameters ( Picture *pic )
{
  m_originalBuf = &pic->getOrigBuffer();                                   // original frame
  m_workingBuf = &pic->getFilteredOrigBuffer();                            // mctf filtered frame

  // Determine blockSize dynamically based on the frame resolution
  int blockSize = BLK_8;
  uint32_t picSizeInLumaSamples = m_workingBuf->Y().height * m_workingBuf->Y().width;
  if ( picSizeInLumaSamples >= 7680 * 4320 )
  {
    // 8K resolution
    blockSize = BLK_32;
  }
  else if ( picSizeInLumaSamples >= 3840 * 2160 )
  {
    // 4K resolution
    blockSize = BLK_16;
  }
  else
  {
    blockSize = BLK_8;
  }

  findMask( COMP_Y );                                                       // Generate mask for luma only

  // find difference between original and filtered/reconstructed frame => film grain estimate
  m_grainEstimateBuf->subtract( pic->getOrigBuffer(),
                                pic->getFilteredOrigBuffer() );

  for ( int compIdx = 0; compIdx < getNumberValidComponents( m_inputChromaFormat ); compIdx++ )
  {
    ComponentID  compID          = ComponentID( compIdx );
    uint32_t     width           = m_workingBuf->getBuf( compID ).width;    // Width of current frame
    uint32_t     height          = m_workingBuf->getBuf( compID ).height;   // Height of current frame
    uint32_t     windowSize      = DATA_BASE_SIZE;                          // Size for Film Grain block
    int          bitDepth        = m_bitDepths[toChannelType( compID )];
    int          detect_edges    = 0;
    int          mean            = 0;
    int          var             = 0;
    m_numDctGrainBlocks          = 0;

    // Clear vectors before computing for each component
    vecMean.clear();
    vecVar.clear();
    tmp_data_x.clear();
    tmp_data_y.clear();
    scalingVec.clear();
    quantVec.clear();
    coeffs.clear();

    for ( int i = 0; i <= width - windowSize; i += windowSize )
    { // loop over windowSize x windowSize blocks
      for ( int j = 0; j <= height - windowSize; j += windowSize )
      {
        if ( compID == COMP_Y )
        {
          detect_edges = countEdges ( windowSize,
                                      i,
                                      j,
                                      compID );  // for flat region without edges
        }
        else
        {
          detect_edges = 1;                      // always process for chroma
        }
        if ( detect_edges )   // selection of uniform, flat and low-complexity area; extend to other features, e.g., variance.
        { // find transformed blocks; cut-off frequency estimation is done on 64 x 64 blocks as low-pass filtering on synthesis side is done on 64 x 64 blocks.
          CoeffBuf& currentCoeffBuf = m_dctGrainBlockList[m_numDctGrainBlocks++];
          blockTransform ( currentCoeffBuf,
                           i,
                           j,
                           bitDepth,
                           compID );
        }

        int step = windowSize / blockSize;
        for ( int k = 0; k < step; k++ )
        {
          for ( int m = 0; m < step; m++ )
          {
            if ( compID == COMP_Y )
            {
              detect_edges = countEdges ( blockSize,
                                          i + k * blockSize,
                                          j + m * blockSize,
                                          compID );   // for flat region without edges
            }
            else
            {
              detect_edges = 1;  // always process for chroma
            }
            if ( detect_edges )   // selection of uniform, flat and low-complexity area; extend to other features, e.g., variance.
            {
              // collect all data for parameter estimation; mean and variance are caluclated on blockSize x blockSize blocks
              uint32_t stride = m_grainEstimateBuf->get( compID ).stride;
              double varD = calcVar ( m_grainEstimateBuf->get( compID ).buf + ( ( j + m * blockSize ) * stride ) + i + ( k * blockSize ),
                                      stride,
                                      blockSize,
                                      blockSize );
              varD = varD / (( blockSize * blockSize ));
              var = static_cast<int>( varD + 0.5 );
              stride = m_workingBuf->get( compID ).stride;
              mean = calcMean ( m_workingBuf->get( compID ).buf + ( ( j + m * blockSize ) * stride ) + i + ( k * blockSize ),
                                stride,
                                blockSize,
                                blockSize );
              mean = static_cast<int>(static_cast<double>( mean ) / ( blockSize * blockSize ) + 0.5 );

              // regularize high variations; controls excessively fluctuating points
              double tmp = 2.75 * pow( static_cast<double>( var ), 0.5 ) + 0.5;
              var = static_cast<int>( tmp ); 
              // limit data points to meaningful values. higher variance can be result of not perfect mask estimation (non-flat regions fall in estimation process)
              if ( var < ( MAX_REAL_SCALE << ( bitDepth - BIT_DEPTH_8 ) ) )
              {
                vecMean.push_back( mean );    // mean of the filtered frame
                vecVar.push_back( var );      // variance of the film grain estimate
              }
            }
          }
        }
      }
    }

    // calculate film grain parameters
    estimateCutoffFreqAdaptive( compID );
    estimateScalingFactors ( bitDepth,
                             compID );

    // Clear vectors after estimation
    vecMean.clear();
    vecVar.clear();
    finalIntervalsandScalingFactors.clear();
  }
}

/* This function calculates the scaling factors for film grain by analyzing the variance of intensity intervals.
 * The primary steps include fitting a polynomial regression function to the intensity - variance data points,
 * smoothing the resulting scaling function, and performing Lloyd - Max quantization to derive the final scaling factors.
 * The estimated parameters are then set for each intensity interval.*/
void FGAnalyzer::estimateScalingFactors ( uint32_t bitDepth,
                                          ComponentID compId )
{
  // if cutoff frequencies are not estimated previously, do not proceed since presentFlag is set to false in a previous step
  if ( !m_compModel[compId].presentFlag || vecMean.size() < MIN_POINTS_FOR_INTENSITY_ESTIMATION )
  {
    return;   // If there is no enough points to estimate film grain intensities, default or previously estimated
              // parameters are used
  }

  double              distortion = 0.0;

  // Fit the points with the curve and perform Lloyd Max quantization.
  bool valid;
  for ( int i = 0; i < NUM_PASSES; i++ )   // if num_passes = 2, filtering of the dataset points is performed
  {
    valid = fitFunction ( ORDER,
                          bitDepth,
                          i );   // n-th order polynomial regression for scaling function estimation
    if ( !valid )
    {
      coeffs.clear();
      scalingVec.clear();
      quantVec.clear();
      break;
    }
  }

  if ( valid )
  {
    avgScalingVec ( bitDepth );   // scale with previously fitted function to smooth the intensity
    valid = lloydMax ( distortion,
                       bitDepth );   // train quantizer and quantize curve using Lloyd Max
  }

  // Based on quantized intervals, set intensity region and scaling parameter
  if ( valid )   // if not valid, reuse previous parameters (for example, if var is all zero)
  {
    setEstimatedParameters ( bitDepth,
                             compId );
  }

  coeffs.clear();
  scalingVec.clear();
  quantVec.clear();

}

/*This function divides the specified range(rows or columns) of the `meanSquaredDctGrain` matrix into bins
* and calculates the average value of each bin.If the average value of a bin exceeds the given threshold,
* the bin is considered significant and its starting index is recorded in the `significantIndices` vector.
* The function can be used to adaptively refine the search for significant values in the matrix by focusing
* on specific rows or columns iteratively.*/
void FGAnalyzer::adaptiveSampling ( int bins,
                                    double threshold,
                                    std::vector<int>& significantIndices,
                                    int startIdx,
                                    bool isRow )
{
  int binSize = DATA_BASE_SIZE / bins;
  for ( int i = 0; i < bins; i++ )
  {
    double sum = 0;
    for ( int j = 0; j < binSize; j++ )
    {
      int idx = startIdx + i * binSize + j;
      if ( idx >= DATA_BASE_SIZE )
          break;  // Ensure we don't go out of bounds
      sum += isRow ? meanSquaredDctGrain[idx][0] : meanSquaredDctGrain[0][idx];
    }
    sum /= binSize;
    if ( sum > threshold )
    {
      significantIndices.push_back( startIdx + i * binSize );
    }
  }
}


/*This function refines the cutoff frequency estimation by adaptively sampling the mean squared DCT grain values
 * matrix. Instead of analyzing every row and column, it focuses on significant bins determined by the adaptive sampling
 * method. The horizontal and vertical cutoff frequencies are estimated by examining the mean values of these significant
 * bins, making the process more efficient and reducing computational overhead.
 * The function performs the following steps :
 * 1. Initializes mean squared DCT grain matrix and mean vectors for rows and columns.
 * 2. Iterates through the DCT grain blocks to calculate the average block for each coefficient.
 * 3. Uses the adaptive sampling method to identify significant rows and columns.
 * 4. Estimates the cutoff frequencies based on the mean values of the significant rows and columns.
 * 5. Updates the component model with the estimated cutoff frequencies.*/
void FGAnalyzer::estimateCutoffFreqAdaptive( ComponentID compId )
{
  const int coarseBins = 8; // Initial coarse sampling bins
  const int refineBins = 4; // Bins for each refinement step
  const int maxIterations = 3; // Maximum refinement iterations
  const double threshold = 0.1; // Threshold to identify significant bins

  std::memset( meanSquaredDctGrain, 0, sizeof( meanSquaredDctGrain ) );

  // Calculate mean squared DCT grain values
  for ( int x = 0; x < DATA_BASE_SIZE; x++ )
  {
    for ( int y = 0; y < DATA_BASE_SIZE; y++ )
    {
      for ( int i = 0; i < m_numDctGrainBlocks; i++ )
      {
        meanSquaredDctGrain[x][y] += m_dctGrainBlockList[i].at( x, y );
      }
      meanSquaredDctGrain[x][y] /= m_numDctGrainBlocks;
    }
  }

  // Identify initial coarse bins with significant grain values
  std::vector<int> significantRows, significantCols;
  adaptiveSampling ( coarseBins,
                     threshold,
                     significantRows,
                     true ); // Rows
  adaptiveSampling ( coarseBins,
                     threshold,
                     significantCols,
                     false );  // Columns

  // Iterative Refinement
  for ( int iter = 0; iter < maxIterations; iter++ )
  {
    std::vector<int> refinedRows, refinedCols;
    for ( int row : significantRows )
    {
      adaptiveSampling ( refineBins,
                         threshold,
                         refinedRows,
                         row,
                         true );
    }
    for ( int col : significantCols )
    {
      adaptiveSampling ( refineBins,
                         threshold,
                         refinedCols,
                         col,
                         false );
    }
    significantRows = refinedRows;
    significantCols = refinedCols;
  }

  // Determine cut-off frequencies from the refined significant bins
  int cutoffVertical = significantRows.empty() ? 0 : significantRows.back() / ( DATA_BASE_SIZE / 16 );
  int cutoffHorizontal = significantCols.empty() ? 0 : significantCols.back() / ( DATA_BASE_SIZE / 16 );

  // Set the cut-off frequencies in the model
  if ( cutoffVertical && cutoffHorizontal )
  {
    m_compModel[compId].presentFlag = true;
    m_compModel[compId].numModelValues = 3;
    m_compModel[compId].intensityValues[0].compModelValue[1] = cutoffHorizontal;
    m_compModel[compId].intensityValues[0].compModelValue[2] = cutoffVertical;
  }
  else
  {
    m_compModel[compId].presentFlag = false;
  }
}

// DCT-2 64x64 as defined in VVC
void FGAnalyzer::blockTransform ( CoeffBuf &currentCoeffBuf,
                                  int offsetX,
                                  int offsetY,
                                  uint32_t bitDepth,
                                  ComponentID compId )
{
  uint32_t      windowSize      = DATA_BASE_SIZE;   // Size for Film Grain block
  const int     transform_scale = 9;                // upscaling of original transform as specified in VVC (for 64x64 block)

  // copy input -> 32 Bit
  for ( uint32_t y = 0; y < DATA_BASE_SIZE; y++ )
  {
    for ( uint32_t x = 0; x < DATA_BASE_SIZE; x++ )
    {
      m_DCTinout[x + DATA_BASE_SIZE * y] = m_grainEstimateBuf->get( compId ).at( offsetX + x,
                                                                                 offsetY + y );
    }
  }

  fastForwardDCT2_B64 ( m_DCTinout,
                        m_DCTtemp,
                        transform_scale,
                        windowSize,
                        0,
                        0 );
  fastForwardDCT2_B64 ( m_DCTtemp,
                        m_DCTinout,
                        transform_scale,
                        windowSize,
                        0,
                        0 );

  // Calculate squared transformed block
  for ( int y = 0; y < DATA_BASE_SIZE; y++ )
  {
    for ( int x = 0; x < DATA_BASE_SIZE; x++ )
    {
      currentCoeffBuf.at( x, y ) = m_DCTinout[x + DATA_BASE_SIZE * y] * m_DCTinout[x + DATA_BASE_SIZE * y];
    }
  }
}

// check edges
int FGAnalyzer::countEdges ( int windowSize,
                             int offsetX,
                             int offsetY, 
                             ComponentID compId )
{
  for ( int x = 0; x < windowSize; x++ )
  {
    for ( int y = 0; y < windowSize; y++ )
    {
      if ( m_maskBuf->get( compId ).at( offsetX + x,
                                        offsetY + y ) )
      {
        return 0;
      }
    }
  }

  return 1;
}

// Fit data to a function using n-th order polynomial interpolation
bool FGAnalyzer::fitFunction ( int order,
                               int bitDepth,
                               bool second_pass )
{
  long double         a[MAXPAIRS + 1][MAXPAIRS + 1];
  long double         B[MAXPAIRS + 1], C[MAXPAIRS + 1], S[MAXPAIRS + 1];
  long double         A1 = 0.0, A2 = 0.0, Y1 = 0.0, m = 0.0, S1 = 0.0, x1 = 0.0;
  long double         xscale = 0.0, yscale = 0.0;
  long double         xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
  long double         polycoefs[MAXORDER + 1];
  int i, j, k, L, R;

  // several data filtering and data manipulations before fitting the function
  // create interval points for function fitting
  int INTENSITY_INTERVAL_NUMBER = (1 << bitDepth) / INTERVAL_SIZE;
  vec_mean_intensity.resize( INTENSITY_INTERVAL_NUMBER, 0 );
  vec_variance_intensity.resize( INTENSITY_INTERVAL_NUMBER, 0 );
  element_number_per_interval.resize( INTENSITY_INTERVAL_NUMBER, 0 );

  double              mn = 0.0, sd = 0.0;

  std::memset( a, 0, sizeof(a) );
  std::memset( B, 0, sizeof(B) );
  std::memset( C, 0, sizeof(C) );
  std::memset( S, 0, sizeof(S) );
  std::memset( polycoefs, 0, sizeof(polycoefs) );

  if ( second_pass )   // in second pass, filter based on the variance of the data_y. remove all high and low points
  {
    xmin = scalingVec.back();
    scalingVec.pop_back();
    xmax = scalingVec.back();
    scalingVec.pop_back();
    int n = static_cast<int>( vecVar.size() );
    if ( n != 0 )
    {
      mn = std::accumulate ( vecVar.begin(), vecVar.end(), 0.0 ) / n;
      for ( int cnt = 0; cnt < n; cnt++ )
      {
        sd += ( vecVar[cnt] - mn ) * ( vecVar[cnt] - mn );
      }
      sd /= n;
      sd = std::sqrt( sd );
    }
  }

  for ( int cnt = 0; cnt < vecMean.size(); cnt++ )
  {
    if ( second_pass )
    {
      if ( vecMean[cnt] >= xmin && vecMean[cnt] <= xmax )
      {
        if (( vecVar[cnt] < scalingVec[vecMean[cnt] - static_cast<int>(xmin)] + sd * VAR_SCALE_UP ) && ( vecVar[cnt] > scalingVec[vecMean[cnt] - static_cast<int>(xmin)] - sd * VAR_SCALE_DOWN ))
        {
          int block_index = vecMean[cnt] / INTERVAL_SIZE;
          vec_mean_intensity[block_index] += vecMean[cnt];
          vec_variance_intensity[block_index] += vecVar[cnt];
          element_number_per_interval[block_index]++;
        }
      }
    }
    else
    {
      int block_index = vecMean[cnt] / INTERVAL_SIZE;
      vec_mean_intensity[block_index] += vecMean[cnt];
      vec_variance_intensity[block_index] += vecVar[cnt];
      element_number_per_interval[block_index]++;
    }
  }

  // create points per intensity interval
  for ( int block_idx = 0; block_idx < INTENSITY_INTERVAL_NUMBER; block_idx++ )
  {
    if ( element_number_per_interval[block_idx] >= MIN_ELEMENT_NUMBER_PER_INTENSITY_INTERVAL )
    {
      tmp_data_x.push_back ( vec_mean_intensity[block_idx] / element_number_per_interval[block_idx] );
      tmp_data_y.push_back( vec_variance_intensity[block_idx] / element_number_per_interval[block_idx] );
    }
  }

  // There needs to be at least ORDER+1 points to fit the function
  if ( tmp_data_x.size() < ( order + 1 ) )
  {
    return false;   // if there is no enough blocks to estimate film grain parameters, default or previously estimated
                    // parameters are used
  }

  for ( i = 0; i < tmp_data_x.size(); i++ ) // remove single points before extending and fitting
  {
    int check = 0;
    for ( j = -WINDOW; j <= WINDOW; j++ )
    {
      int idx = i + j;
      if ( idx >= 0 && idx < tmp_data_x.size() && j != 0 )
      {
        check += abs( tmp_data_x[i] / INTERVAL_SIZE - tmp_data_x[idx] / INTERVAL_SIZE ) <= WINDOW ? 1 : 0;
      }
    }

    if ( check < NBRS )
    {
      for ( int k = i; k < tmp_data_x.size() - 1; k++ )
      {
        tmp_data_x[k] = tmp_data_x[k + 1];
        tmp_data_y[k] = tmp_data_y[k + 1];
      }
      tmp_data_x.pop_back();
      tmp_data_y.pop_back();
      i--;
    }
  }

  extendPoints( bitDepth );     // find the most left and the most right point, and extend edges

  CHECK( tmp_data_x.size() > MAXPAIRS, "Maximum dataset size exceeded." );

  // fitting the function starts here
  xmin = tmp_data_x[0];
  xmax = tmp_data_x[0];
  ymin = tmp_data_y[0];
  ymax = tmp_data_y[0];
  for ( i = 0; i < tmp_data_x.size(); i++ )
  {
    if ( tmp_data_x[i] < xmin )
    {
      xmin = tmp_data_x[i];
    }
    if ( tmp_data_x[i] > xmax )
    {
      xmax = tmp_data_x[i];
    }
    if ( tmp_data_y[i] < ymin )
    {
      ymin = tmp_data_y[i];
    }
    if ( tmp_data_y[i] > ymax )
    {
      ymax = tmp_data_y[i];
    }
  }

  long double xlow = xmax;
  long double ylow = ymax;

  int data_pairs = static_cast<int>( tmp_data_x.size() );

  double data_array[2][MAXPAIRS + 1];
  std::memset( data_array, 0, sizeof(data_array) );
  for ( i = 0; i < data_pairs; i++ )
  {
    data_array[0][i + 1] = static_cast<double>( tmp_data_x[i] );
    data_array[1][i + 1] = static_cast<double>( tmp_data_y[i] );
  }

  // Clear previous vectors by resizing them to 0
  tmp_data_x.clear();
  tmp_data_y.clear();

  if ( second_pass )
  {
    coeffs.resize( 0 );
    scalingVec.resize( 0 );
  }

  for ( i = 1; i <= data_pairs; i++ )
  {
    if ( data_array[0][i] < xlow && data_array[0][i] != 0 )
    {
      xlow = data_array[0][i];
    }
    if ( data_array[1][i] < ylow && data_array[1][i] != 0 )
    {
      ylow = data_array[1][i];
    }
  }

  if ( xlow < .001 && xmax < 1000 )
  {
    xscale = 1 / xlow;
  }
  else if ( xmax > 1000 && xlow > .001 )
  {
    xscale = 1 / xmax;
  }
  else
  {
    xscale = 1;
  }

  if ( ylow < .001 && ymax < 1000 )
  {
    yscale = 1 / ylow;
  }
  else if ( ymax > 1000 && ylow > .001 )
  {
    yscale = 1 / ymax;
  }
  else
  {
    yscale = 1;
  }

  // initialise array variables
  for ( j = 1; j <= data_pairs; j++ )
  {
    for ( i = 1; i <= order; i++ )
    {
      B[i] = B[i] + data_array[1][j] * yscale * ldpow( data_array[0][j] * xscale, i );
      if ( B[i] == std::numeric_limits<long double>::max() )
      {
        return false;
      }
      for ( k = 1; k <= order; k++ )
      {
        a[i][k] = a[i][k] + ldpow( data_array[0][j] * xscale, ( i + k ) );
        if ( a[i][k] == std::numeric_limits<long double>::max() )
        {
          return false;
        }
      }
      S[i] = S[i] + ldpow( data_array[0][j] * xscale, i );
      if ( S[i] == std::numeric_limits<long double>::max() )
      {
        return false;
      }
    }
    Y1 = Y1 + data_array[1][j] * yscale;
    if ( Y1 == std::numeric_limits<long double>::max() )
    {
      return false;
    }
  }

  for ( i = 1; i <= order; i++ )
  {
    for ( j = 1; j <= order; j++ )
    {
      a[i][j] = a[i][j] - S[i] * S[j] / static_cast<long double>( data_pairs );
      if (a[i][j] == std::numeric_limits<long double>::max())
      {
        return false;
      }
    }
    B[i] = B[i] - Y1 * S[i] / static_cast<long double>( data_pairs );
    if ( B[i] == std::numeric_limits<long double>::max() )
    {
      return false;
    }
  }

  for ( k = 1; k <= order; k++ )
  {
    R  = k;
    A1 = 0;
    for ( L = k; L <= order; L++ )
    {
      A2 = fabsl( a[L][k] );
      if ( A2 > A1 )
      {
        A1 = A2;
        R  = L;
      }
    }
    if ( A1 == 0 )
    {
      return false;
    }
    if ( R != k )
    {
      for ( j = k; j <= order; j++ )
      {
        x1      = a[R][j];
        a[R][j] = a[k][j];
        a[k][j] = x1;
      }
      x1   = B[R];
      B[R] = B[k];
      B[k] = x1;
    }
    for ( i = k; i <= order; i++ )
    {
      m = a[i][k];
      for ( j = k; j <= order; j++ )
      {
        if ( i == k )
        {
          a[i][j] = a[i][j] / m;
        }
        else
        {
          a[i][j] = a[i][j] - m * a[k][j];
        }
      }
      if ( i == k )
      {
        B[i] = B[i] / m;
      }
      else
      {
        B[i] = B[i] - m * B[k];
      }
    }
  }

  polycoefs[order] = B[order];
  for ( k = 1; k <= order - 1; k++ )
  {
    i  = order - k;
    S1 = 0;
    for ( j = 1; j <= order; j++ )
    {
      S1 = S1 + a[i][j] * polycoefs[j];
      if ( S1 == std::numeric_limits<long double>::max() )
      {
        return false;
      }
    }
    polycoefs[i] = B[i] - S1;
  }

  S1 = 0;
  for ( i = 1; i <= order; i++ )
  {
    S1 = S1 + polycoefs[i] * S[i] / static_cast<long double>( data_pairs );
    if ( S1 == std::numeric_limits<long double>::max() )
    {
      return false;
    }
  }
  polycoefs[0] = (Y1 / static_cast<long double>( data_pairs ) - S1);

  // zero all coeficient values smaller than +/- .00000000001 (avoids -0)
  for ( i = 0; i <= order; i++ )
  {
    if ( fabsl(polycoefs[i] * 100000000000) < 1 )
    {
      polycoefs[i] = 0;
    }
  }

  // rescale parameters
  for ( i = 0; i <= order; i++ )
  {
    polycoefs[i] = (1 / yscale) * polycoefs[i] * ldpow( xscale, i );
    coeffs.push_back( polycoefs[i] );
  }

  // create fg scaling function. interpolation based on coeffs which returns lookup table from 0 - 2^B-1. n-th order polinomial regression
  for ( i = static_cast<int>( xmin ); i <= static_cast<int>( xmax ); i++ )
  {
    double val = coeffs[0];
    for ( j = 1; j < coeffs.size(); j++ )
    {
      val += (coeffs[j] * ldpow( i, j ));
    }

    val = Clip3( 0.0,
                 static_cast<double>( 1 << bitDepth ) - 1,
                 val );
    scalingVec.push_back( val );
  }

  // save in scalingVec min and max value for further use
  scalingVec.push_back( xmax );
  scalingVec.push_back( xmin );

  vec_mean_intensity.clear();
  vec_variance_intensity.clear();
  element_number_per_interval.clear();
  tmp_data_x.clear();
  tmp_data_y.clear();

  return true;
}

// avg scaling vector with previous result to smooth transition betweeen frames
void FGAnalyzer::avgScalingVec ( int bitDepth )
{
  int xmin = static_cast<int>( scalingVec.back() );
  scalingVec.pop_back();
  int xmax = static_cast<int>( scalingVec.back() );
  scalingVec.pop_back();

  std::vector<double> scalingVecAvg( static_cast<int>( 1 << bitDepth ) );
  bool isFirstScalingEst = true;

  if ( isFirstScalingEst )
  {
    for (int i = xmin; i <= xmax; i++)
    {
      scalingVecAvg[i] = scalingVec[i - xmin];
    }
    isFirstScalingEst = false;
  }
  else
  {
    for ( int i = xmin; i <= xmax; i++ )
    {
      scalingVecAvg[i] = ( scalingVecAvg[i] + scalingVec[i - xmin] ) / 2.0;
    }
  }

  int new_xmin = 0;
  while ( new_xmin <= xmax && scalingVecAvg[new_xmin] == 0 )
  {
    new_xmin++;
  }

  int new_xmax = static_cast<int>( scalingVecAvg.size() ) - 1;
  while ( new_xmax >= 0 && scalingVecAvg[new_xmax] == 0 )
  {
    new_xmax--;
  }

  if ( new_xmax < new_xmin )
  {
    // Handle the case where all entries are zero
    scalingVec.clear();
    scalingVec.push_back( 0 ); // Minimum value
    scalingVec.push_back( 0 ); // Maximum value
    return;
  }

  scalingVec.assign( scalingVecAvg.begin() + new_xmin,
                     scalingVecAvg.begin() + new_xmax + 1 );
  scalingVec.push_back( new_xmax );
  scalingVec.push_back( new_xmin );
}


// Lloyd Max quantizer
bool FGAnalyzer::lloydMax ( double &distortion,
                            int bitDepth )
{
  if ( !scalingVec.size() )
  {
    // Film grain parameter estimation is not performed. Default or previously estimated parameters are reused.
    return false;
  }

  int xmin = static_cast<int>( scalingVec.back() );
  scalingVec.pop_back();
  scalingVec.pop_back();   // dummy pop_pack ==> int xmax = (int)scalingVec.back();

  double ymin          = 0.0;
  double ymax          = 0.0;
  double init_training = 0.0;
  double tolerance     = 0.0000001;
  double last_distor   = 0.0;
  double rel_distor    = 0.0;

  double codebook[QUANT_LEVELS];
  double partition[QUANT_LEVELS - 1];

  std::vector<double> tmpVec( scalingVec.size(), 0.0 );
  distortion = 0.0;

  ymin = scalingVec[0];
  ymax = scalingVec[0];
  for ( int i = 0; i < scalingVec.size(); i++ )
  {
    if ( scalingVec[i] < ymin )
    {
      ymin = scalingVec[i];
    }
    if ( scalingVec[i] > ymax )
    {
      ymax = scalingVec[i];
    }
  }

  init_training = ( ymax - ymin ) / QUANT_LEVELS;

  if ( init_training <= 0 )
  {
    // msg(WARNING, "Invalid training dataset. Film grain parameter estimation is not performed. Default or previously estimated parameters are reused.\n");
    return false;
  }

  // initial codebook
  double step = init_training / 2;
  for ( int i = 0; i < QUANT_LEVELS; i++ )
  {
    codebook[i] = ymin + i * init_training + step;
  }

  // initial partition
  for ( int i = 0; i < QUANT_LEVELS - 1; i++ )
  {
    partition[i] = (codebook[i] + codebook[i + 1]) / 2;
  }

  // quantizer initialization
  quantize ( tmpVec,
             distortion,
             partition,
             codebook );

  double tolerance2 = std::numeric_limits<double>::epsilon() * ymax;
  if ( distortion > tolerance2 )
  {
    rel_distor = abs( distortion - last_distor ) / distortion;
  }
  else
  {
    rel_distor = distortion;
  }

  // optimization: find optimal codebook and partition
  while ( ( rel_distor > tolerance ) && ( rel_distor > tolerance2 ) )
  {
    for ( int i = 0; i < QUANT_LEVELS; i++ )
    {
      int count = 0;
      double sum = 0.0;

      for ( int j = 0; j < tmpVec.size(); j++ )
      {
        if ( codebook[i] == tmpVec[j] )
        {
          count++;
          sum += scalingVec[j];
        }
      }

      if ( count )
      {
        codebook[i] = sum / static_cast<double>( count );
      }
      else
      {
        sum   = 0.0;
        count = 0;
        if ( i == 0 )
        {
          for ( int j = 0; j < tmpVec.size(); j++ )
          {
            if ( scalingVec[j] <= partition[i] )
            {
              count++;
              sum += scalingVec[j];
            }
          }
          if ( count )
          {
            codebook[i] = sum / static_cast<double>( count );
          }
          else
          {
            codebook[i] = ( partition[i] + ymin ) / 2;
          }
        }
        else if ( i == QUANT_LEVELS - 1 )
        {
          for ( int j = 0; j < tmpVec.size(); j++ )
          {
            if (scalingVec[j] >= partition[i - 1])
            {
              count++;
              sum += scalingVec[j];
            }
          }
          if ( count )
          {
            codebook[i] = sum / static_cast<double>( count );
          }
          else
          {
            codebook[i] = ( partition[i - 1] + ymax ) / 2;
          }
        }
        else
        {
          for ( int j = 0; j < tmpVec.size(); j++ )
          {
            if ( scalingVec[j] >= partition[i - 1] && scalingVec[j] <= partition[i] )
            {
              count++;
              sum += scalingVec[j];
            }
          }
          if ( count )
          {
            codebook[i] = sum / static_cast<double>( count );
          }
          else
          {
            codebook[i] = ( partition[i - 1] + partition[i] ) / 2;
          }
        }
      }
    }

    // compute and sort partition
    for ( int i = 0; i < QUANT_LEVELS - 1; i++ )
    {
      partition[i] = ( codebook[i] + codebook[i + 1] ) / 2;
    }
    std::sort( partition, partition + QUANT_LEVELS - 1 );

    // final quantization - testing condition
    last_distor = distortion;
    quantize ( tmpVec,
               distortion,
               partition,
               codebook );

    if ( distortion > tolerance2 )
    {
      rel_distor = abs( distortion - last_distor ) / distortion;
    }
    else
    {
      rel_distor = distortion;
    }
  }

  // fill the final quantized vector
  int maxVal = ( 1 << bitDepth ) - 1;  // Full range max value for given bit depth
  quantVec.resize( static_cast<int>( 1 << bitDepth ), 0 );
  for ( int i = 0; i < tmpVec.size(); i++ )
  {
    quantVec[i + xmin] = Clip3( 0, 
                                maxVal,                                    
                                static_cast<int>( tmpVec[i] + 0.5 ) );
  }

  return true;
}

void FGAnalyzer::quantize ( std::vector<double>& quantizedVec,
                            double& distortion,
                            double partition[],
                            double codebook[] )
{
  // Reset previous quantizedVec to 0 and distortion to 0
  std::fill(quantizedVec.begin(), quantizedVec.end(), 0.0);
  distortion = 0.0;

  // Quantize input vector
  for ( int i = 0; i < scalingVec.size(); i++ )
  {
    double quantizedValue = 0.0;
    for ( int j = 0; j < QUANT_LEVELS - 1; j++ )
    {
      quantizedValue += ( scalingVec[i] > partition[j] );
    }
    quantizedVec[i] = codebook[static_cast<int>( quantizedValue )];
  }

  // Compute distortion (MSE)
  for ( int i = 0; i < scalingVec.size(); i++ )
  {
    double error = scalingVec[i] - quantizedVec[i];
    distortion += ( error * error );
  }
  distortion /= scalingVec.size();
}

// Set correctlly SEI parameters based on the quantized curve
void FGAnalyzer::setEstimatedParameters ( uint32_t bitDepth,
                                          ComponentID compId )
{
  // calculate intervals and scaling factors
  defineIntervalsAndScalings ( bitDepth );

  // Merge small intervals with left or right interval
  for ( size_t i = 0; i < finalIntervalsandScalingFactors.size(); ++i )
  {
    int tmp1 = finalIntervalsandScalingFactors[i][1] - finalIntervalsandScalingFactors[i][0];

    if ( tmp1 < ( 2 << ( bitDepth - BIT_DEPTH_8 ) ) )
    {
      int diffRight = ( i == finalIntervalsandScalingFactors.size() - 1 ) || ( finalIntervalsandScalingFactors[i + 1][2] == 0 )
          ? std::numeric_limits<int>::max()
          : abs( finalIntervalsandScalingFactors[i][2] - finalIntervalsandScalingFactors[i + 1][2] );
      int diffLeft = ( i == 0 ) || ( finalIntervalsandScalingFactors[i - 1][2] == 0 )
          ? std::numeric_limits<int>::max()
          : abs( finalIntervalsandScalingFactors[i][2] - finalIntervalsandScalingFactors[i - 1][2] );

      if ( diffLeft < diffRight )
      {
        int tmp2 = finalIntervalsandScalingFactors[i - 1][1] - finalIntervalsandScalingFactors[i - 1][0];
        int newScale = ( tmp2 * finalIntervalsandScalingFactors[i - 1][2] + tmp1 * finalIntervalsandScalingFactors[i][2] ) / ( tmp2 + tmp1 );

        finalIntervalsandScalingFactors[i - 1][1] = finalIntervalsandScalingFactors[i][1];
        finalIntervalsandScalingFactors[i - 1][2] = newScale;
        finalIntervalsandScalingFactors.erase( finalIntervalsandScalingFactors.begin() + i );
        --i;
      }
      else
      {
        int tmp2 = finalIntervalsandScalingFactors[i + 1][1] - finalIntervalsandScalingFactors[i + 1][0];
        int newScale = ( tmp2 * finalIntervalsandScalingFactors[i + 1][2] + tmp1 * finalIntervalsandScalingFactors[i][2] ) / ( tmp2 + tmp1 );

        finalIntervalsandScalingFactors[i][1] = finalIntervalsandScalingFactors[i + 1][1];
        finalIntervalsandScalingFactors[i][2] = newScale;
        finalIntervalsandScalingFactors.erase( finalIntervalsandScalingFactors.begin() + i + 1 );
        --i;
      }
    }
  }

  // scale to 8-bit range as supported by current sei and rdd5
  scaleDown ( bitDepth );

  // because of scaling in previous step, some intervals may overlap. Check intervals for errors.
  confirmIntervals ( );

  // Set number of intervals; exclude intervals with scaling factor 0.
  m_compModel[compId].numIntensityIntervals =
      static_cast<uint8_t>( finalIntervalsandScalingFactors.size() - std::count_if ( finalIntervalsandScalingFactors.begin(),
                                                                                     finalIntervalsandScalingFactors.end(),
                                                                                     []( const std::array<int, 3>& interval )
                                                                                     {
                                                                                       return interval[2] == 0;
                                                                                     }
                                                                                   ) );

  // check if all intervals are 0, and if yes set presentFlag to false
  if ( m_compModel[compId].numIntensityIntervals == 0 )
  { 
    m_compModel[compId].presentFlag = false;
    return;
  }

  // Set final interval boundaries and scaling factors.
  // Check if some interval has scaling factor 0, and do not encode them within SEI.
  int j = 0;
  for ( const auto& interval : finalIntervalsandScalingFactors )
  {
    if ( interval[2] != 0 )
    {
      m_compModel[compId].intensityValues[j].intensityIntervalLowerBound = interval[0];
      m_compModel[compId].intensityValues[j].intensityIntervalUpperBound = interval[1];
      m_compModel[compId].intensityValues[j].compModelValue[0] = interval[2];
      m_compModel[compId].intensityValues[j].compModelValue[1] = m_compModel[compId].intensityValues[0].compModelValue[1];
      m_compModel[compId].intensityValues[j].compModelValue[2] = m_compModel[compId].intensityValues[0].compModelValue[2];
      ++j;
    }
  }
  CHECK( j != m_compModel[compId].numIntensityIntervals, "Check film grain intensity levels" );
}

long double FGAnalyzer::ldpow ( long double n,
                                unsigned p )
{
  long double result = 1.0;

  // Handle special cases for p = 0 and p = 1
  if ( p == 0 ) return 1.0;
  if ( p == 1 ) return n;

  // Exponentiation by squaring
  while ( p > 0 )
  {
    if ( p % 2 == 1 )
      result *= n;
    n *= n;
    p /= 2;
  }
  return result;
}

// find bounds of intensity intervals and scaling factors for each interval
void FGAnalyzer::defineIntervalsAndScalings ( int bitDepth )
{
  finalIntervalsandScalingFactors.clear();
  std::array<int, 3> interval = { 0, 0, quantVec[0] };

  for ( int i = 0; i < (1 << bitDepth) - 1; ++i )
  {
    if ( quantVec[i] != quantVec[i + 1] )
    {
      interval[1] = i;
      finalIntervalsandScalingFactors.push_back ( interval );
      interval[0] = i + 1;
      interval[2] = quantVec[i + 1];
    }
  }
  interval[1] = ( 1 << bitDepth ) - 1;
  finalIntervalsandScalingFactors.push_back ( interval );
}

// scale everything to 8-bit ranges as supported by SEI message
void FGAnalyzer::scaleDown ( int bitDepth )
{
  for ( auto& interval : finalIntervalsandScalingFactors )
  {
    interval[0] >>= ( bitDepth - BIT_DEPTH_8 );
    interval[1] >>= ( bitDepth - BIT_DEPTH_8 );
    interval[2] <<= m_log2ScaleFactor;
    interval[2] >>= ( bitDepth - BIT_DEPTH_8 );
  }
}

// check if intervals are properly set after scaling to 8-bit representation
void FGAnalyzer::confirmIntervals ( )
{
  for ( size_t i = 0; i < finalIntervalsandScalingFactors.size() - 1; ++i )
  {
    if ( finalIntervalsandScalingFactors[i][1] >= finalIntervalsandScalingFactors[i + 1][0] )
    {
      finalIntervalsandScalingFactors[i][1] = finalIntervalsandScalingFactors[i + 1][0] - 1;
    }
  }
}

void FGAnalyzer::extendPoints ( int bitDepth )
{
  int xmin = tmp_data_x[0];
  int xmax = tmp_data_x[0];
  int ymin = tmp_data_y[0];
  int ymax = tmp_data_y[0];
  for ( int i = 0; i < tmp_data_x.size(); i++ )
  {
    if ( tmp_data_x[i] < xmin )
    {
      xmin = tmp_data_x[i];
      ymin = tmp_data_y[i];   // not real ymin
    }
    if ( tmp_data_x[i] > xmax )
    {
      xmax = tmp_data_x[i];
      ymax = tmp_data_y[i];   // not real ymax
    }
  }

  // extend points to the left
  int    step = POINT_STEP;
  double scale = POINT_SCALE;
  int num_extra_point_left = MAX_NUM_POINT_TO_EXTEND;
  int num_extra_point_right = MAX_NUM_POINT_TO_EXTEND;
  while ( xmin >= step && ymin > 1 && num_extra_point_left > 0 )
  {
    xmin -= step;
    ymin = static_cast<int>( ymin / scale );
    tmp_data_x.push_back( xmin );
    tmp_data_y.push_back( ymin );
    num_extra_point_left--;
  }

  // extend points to the right
  while ( xmax + step <= ((1 << bitDepth) - 1) && ymax > 1 && num_extra_point_right > 0 )
  {
    xmax += step;
    ymax = static_cast<int>( ymax / scale );
    tmp_data_x.push_back( xmax );
    tmp_data_y.push_back( ymax );
    num_extra_point_right--;
  }

  // filter out points outside the range
  auto isValid = []( int x )
  {
    return x >= MIN_INTENSITY && x <= MAX_INTENSITY;
  };

  std::vector<int> valid_x, valid_y;
  for ( int i = 0; i < tmp_data_x.size(); i++ )
  {
    if ( isValid( tmp_data_x[i] ) )
    {
      valid_x.push_back( tmp_data_x[i] );
      valid_y.push_back( tmp_data_y[i] );
    }
  }
  tmp_data_x = std::move( valid_x );
  tmp_data_y = std::move( valid_y );
}

