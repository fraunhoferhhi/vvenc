/* RL-RC-DoT research adapter. VVenC's Clear BSD License applies. */
#pragma once

#include "EncoderLib/EncStage.h"
#include "EncoderLib/BinEncoder.h"
#include <array>
#include <map>
#include <vector>

namespace vvenc {
namespace rcdot {

constexpr int WIDTH = 480, HEIGHT = 320, COLS = 30, ROWS = 20, CELLS = COLS * ROWS;
using Map = std::array<double, CELLS>;

struct Analysis
{
  Map energy = {}, intra = {}, inter = {};
  std::array<int, CELLS> referenceCell = {};
};

bool enabled();
void reset( const VVEncCfg& cfg );
void storeAnalysis( int poc, int lastPoc, const Analysis& analysis, const Map& propagation );
void emitState( const Slice& slice );
bool controlsQp();
int requestedQp( const Slice& slice, const Area& area );
void recordCu( const CodingUnit& cu );
void finishPicture( const Picture& pic );
void outputPicture( const Picture& pic, const AccessUnitList& au );
void addSyntaxBits( int poc, bool motion, unsigned bits );

class SyntaxScope
{
  BinEncIf& m_encoder;
  int m_poc;
  bool m_active, m_motion;
  unsigned m_before;
public:
  SyntaxScope( BinEncIf& encoder, const Picture& pic, bool motion )
    : m_encoder(encoder), m_poc(pic.poc),
      m_active(enabled() && encoder.isEncoding() && !pic.isPreAnalysis), m_motion(motion),
      m_before(m_active ? encoder.getNumWrittenBits() : 0) {}
  ~SyntaxScope()
  {
    if( m_active ) addSyntaxBits(m_poc, m_motion, m_encoder.getNumWrittenBits() - m_before);
  }
};

class LookAhead : public EncStage
{
  int m_bitDepth;
  std::map<int, Analysis> m_analysis;
  std::vector<Pel> m_previous;
  Analysis analyze( const CPelBuf& source, const std::vector<Pel>& previous );
protected:
  void initPicture( Picture* pic ) override;
  void processPictures( const PicList& pictures, AccessUnitList&, PicList& done, PicList& free ) override;
public:
  explicit LookAhead( int bitDepth ) : m_bitDepth(bitDepth) {}
};

} // namespace rcdot
} // namespace vvenc
