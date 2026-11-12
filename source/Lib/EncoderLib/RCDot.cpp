/* RL-RC-DoT research adapter. VVenC's Clear BSD License applies. */
#include "RCDot.h"
#include "CommonLib/UnitTools.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace vvenc {
namespace rcdot {
namespace {

struct FrameStats
{
  double area = 0, qpSum = 0, intra = 0, inter = 0, skip = 0;
  double motionBits = 0, transformBits = 0, psnr = 0, ssim = 1;
  uint64_t bits = 0;
};

struct PreState { Analysis analysis; Map propagation; int lastPoc; };
struct Session
{
  std::mutex mutex;
  std::map<int, PreState> pre;
  std::map<int, FrameStats> frames;
  std::map<int, Map> actions;
  FrameStats last;
  int historyPoc = -1, decisions = 0, outputs = 0, total = 0, bitDepth = 10;
  uint64_t bits = 0;
  double qpSum = 0, qpArea = 0, fps = 30, target = 0;
  std::ofstream events, cuLog;
};

Session& session() { static Session state; return state; }
const char* setting( const char* key )
{
  const char* value = std::getenv(key);
  return value && *value ? value : nullptr;
}
std::string mode() { const char* value = setting("VVENC_RCDOT_MODE"); return value ? value : ""; }

void writeMap( std::ostream& out, const Map& map )
{
  out << '[';
  for( int i = 0; i < CELLS; ++i ) out << (i ? "," : "") << map[i];
  out << ']';
}

Map readAction( int poc )
{
  Map action = {};
  if( mode() == "zero" ) return action;
  const char* directory = setting("VVENC_RCDOT_ACTION_DIR");
  if( !directory ) throw std::runtime_error("RL-RC-DoT: missing action directory");
  const std::string path = std::string(directory) + "/" + std::to_string(poc) + ".txt";
  // Collectors may wait while the parent optimizes a full PPO rollout.
  // Encoding-event timeouts are separate; a valid CPU PPO update must not
  // kill a paused codec after the former five-minute action timeout.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3600);
  for( ;; )
  {
    std::ifstream input(path);
    if( input )
    {
      int filePoc = -1;
      if( !(input >> filePoc) || filePoc != poc ) throw std::runtime_error("RL-RC-DoT: action POC mismatch");
      for( double& value : action )
        if( !(input >> value) || !std::isfinite(value) || value < -12.00001 || value > 12.00001 )
          throw std::runtime_error("RL-RC-DoT: expected 600 finite delta-QPs in [-12,12]");
      std::string extra;
      if( input >> extra ) throw std::runtime_error("RL-RC-DoT: extra action data");
      return action;
    }
    if( std::chrono::steady_clock::now() >= deadline ) throw std::runtime_error("RL-RC-DoT: action timeout at " + path);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int satd8( const Pel* source, int sourceStride, const Pel* reference, int referenceStride )
{
  int values[64];
  for( int y = 0; y < 8; ++y )
    for( int x = 0; x < 8; ++x ) values[y * 8 + x] = source[y * sourceStride + x] - reference[y * referenceStride + x];
  for( int axis = 0; axis < 2; ++axis )
    for( int line = 0; line < 8; ++line )
      for( int span = 1; span < 8; span *= 2 )
        for( int base = 0; base < 8; base += 2 * span )
          for( int k = 0; k < span; ++k )
          {
            const int first = axis ? (base + k) * 8 + line : line * 8 + base + k;
            const int second = first + (axis ? 8 : 1) * span;
            const int a = values[first], b = values[second];
            values[first] = a + b; values[second] = a - b;
          }
  int total = 0;
  for( int value : values ) total += std::abs(value);
  return (total + 2) / 4;
}

std::vector<Pel> lowres( const CPelBuf& source, int bitDepth )
{
  std::vector<Pel> result(WIDTH * HEIGHT / 4);
  const int shift = bitDepth - 8;
  for( int y = 0; y < HEIGHT / 2; ++y )
    for( int x = 0; x < WIDTH / 2; ++x )
    {
      const int sum = source.at(2*x,2*y) + source.at(2*x+1,2*y) + source.at(2*x,2*y+1) + source.at(2*x+1,2*y+1);
      result[y * (WIDTH/2) + x] = Pel(Clip3(0,255,(sum + (1 << (shift + 1))) >> (shift + 2)));
    }
  return result;
}

double ssim( const CPelBuf& raw, const CPelBuf& recon, int bitDepth )
{
  const double scale = double(1 << (bitDepth - 8));
  double total = 0;
  for( int by = 0; by < HEIGHT; by += 8 )
    for( int bx = 0; bx < WIDTH; bx += 8 )
    {
      double a = 0, b = 0, aa = 0, bb = 0, ab = 0;
      for( int y = 0; y < 8; ++y )
        for( int x = 0; x < 8; ++x )
        {
          const double u = raw.at(bx+x,by+y) / scale, v = recon.at(bx+x,by+y) / scale;
          a += u; b += v; aa += u*u; bb += v*v; ab += u*v;
        }
      a /= 64; b /= 64;
      const double va = std::max(0.0,aa/64-a*a), vb = std::max(0.0,bb/64-b*b), cov = ab/64-a*b;
      total += ((2*a*b+6.5025)*(2*cov+58.5225))/((a*a+b*b+6.5025)*(va+vb+58.5225));
    }
  return total / ((WIDTH/8)*(HEIGHT/8));
}
} // namespace

bool enabled() { return setting("VVENC_RCDOT_MODE") != nullptr; }
bool controlsQp() { return enabled() && mode() != "anchor"; }

void reset( const VVEncCfg& cfg )
{
  if( setting("VVENC_DOT_STATE_FILE") || setting("VVENC_DOT_ACTION_DIR") )
    throw std::runtime_error("RL-RC-DoT: retired DOT_PROXY bridge; use scripts.rl_rc_dot.session and VVENC_RCDOT_MODE");
  if( !enabled() ) return;
  if( mode() != "anchor" && mode() != "zero" && mode() != "live" )
    throw std::runtime_error("RL-RC-DoT: mode must be anchor, zero or live");
  if( cfg.m_SourceWidth != WIDTH || cfg.m_SourceHeight != HEIGHT || cfg.m_CTUSize != 32 ||
      cfg.m_cuQpDeltaSubdiv != 2 || cfg.m_maxParallelFrames != 0 || cfg.m_RCNumPasses != 1 ||
      cfg.m_RCTargetBitrate <= 0 || cfg.m_internalBitDepth[CH_L] != 10 ||
      cfg.m_framesToBeEncoded <= 0 || cfg.m_FrameRate <= 0 || cfg.m_FrameScale <= 0 )
    throw std::runtime_error("RL-RC-DoT: requires 480x320, CTU32, DQP subdiv2, FPP0, one pass, rate control, 10-bit");
  if( setting("VVENC_RL_STATE_FILE") || setting("VVENC_RL_QP_MAP") || setting("VVENC_RL_ACTION_DIR") ||
      setting("VVENC_DOT_STATE_FILE") || setting("VVENC_DOT_ACTION_DIR") )
    throw std::runtime_error("RL-RC-DoT: legacy CTU bridge environment is also enabled");
  if( !setting("VVENC_RCDOT_STATE_FILE") || !setting("VVENC_RCDOT_RECON_DIR") )
    throw std::runtime_error("RL-RC-DoT: state file and reconstruction directory required");
  Session& s = session();
  s.pre.clear(); s.frames.clear(); s.actions.clear(); s.last = FrameStats();
  s.historyPoc = -1; s.decisions = s.outputs = 0; s.bits = 0; s.qpSum = s.qpArea = 0;
  s.total = cfg.m_framesToBeEncoded; s.fps = double(cfg.m_FrameRate)/cfg.m_FrameScale;
  s.target = cfg.m_RCTargetBitrate; s.bitDepth = cfg.m_internalBitDepth[CH_L];
  s.events.close(); s.events.open(setting("VVENC_RCDOT_STATE_FILE"),std::ios::trunc);
  if( !s.events ) throw std::runtime_error("RL-RC-DoT: cannot open event file");
  s.events << std::setprecision(12);
  s.cuLog.close();
  if( setting("VVENC_RCDOT_CU_FILE") ) s.cuLog.open(setting("VVENC_RCDOT_CU_FILE"),std::ios::trunc);
}

void storeAnalysis( int poc, int lastPoc, const Analysis& analysis, const Map& propagation )
{
  std::lock_guard<std::mutex> lock(session().mutex);
  if( !session().pre.emplace(poc,PreState{analysis,propagation,lastPoc}).second )
    throw std::runtime_error("RL-RC-DoT: repeated pre-analysis POC");
}

void emitState( const Slice& slice )
{
  if( !enabled() || slice.pic->isPreAnalysis ) return;
  Session& s = session();
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    const auto it = s.pre.find(slice.poc);
    if( it == s.pre.end() ) throw std::runtime_error("RL-RC-DoT: pre-analysis missing for current POC");
    const PreState& pre = it->second;
    const FrameStats& f = s.last;
    const double average = s.outputs ? s.bits*s.fps/s.outputs : s.target;
    double complexity = 0;
    for( double cost : pre.analysis.intra ) complexity += cost/CELLS;
    if( s.outputs && int(f.area) != WIDTH*HEIGHT )
      throw std::runtime_error("RL-RC-DoT: completed history has invalid CU coverage");
    const std::array<double,16> global = {{ double(slice.sliceQp), double(slice.poc), double(s.bits),
      f.area ? f.qpSum/f.area : double(slice.sliceQp), s.qpArea ? s.qpSum/s.qpArea : double(slice.sliceQp),
      f.area ? f.intra/f.area : 0, f.area ? f.inter/f.area : 0, f.area ? f.skip/f.area : 0,
      f.psnr, f.ssim, f.bits ? f.motionBits/f.bits : 0, f.bits ? f.transformBits/f.bits : 0,
      double(s.decisions)/s.total, std::log(std::max(average,1.0)/s.target),
      slice.isIntra() ? 2.0 : (slice.isInterP() ? 1.0 : 0.0), complexity }};
    Map inverse = {};
    for( int y = 0; y < ROWS; ++y )
      for( int x = 0; x < COLS; ++x )
      {
        const int ctu = (y/2) * slice.pps->pcv->widthInCtus + x/2;
        const int qp = slice.pic->ctuAdaptedQP.empty() ? slice.sliceQp : slice.pic->ctuAdaptedQP.at(ctu);
        inverse[y*COLS+x] = std::pow(2.0,-(qp-4.0)/6.0);
      }
    s.events << "{\"schema\":\"vvenc-rcdot-vvc-adapter/3\",\"kind\":\"state\",\"poc\":" << slice.poc << ",\"step\":" << s.decisions
             << ",\"history_poc\":" << s.historyPoc << ",\"lookahead_last_poc\":" << pre.lastPoc
             << ",\"global\":[";
    for( int i = 0; i < 16; ++i ) s.events << (i ? "," : "") << global[i];
    s.events << "],\"maps\":[";
    writeMap(s.events,pre.analysis.energy); s.events << ','; writeMap(s.events,pre.analysis.intra);
    s.events << ','; writeMap(s.events,pre.propagation); s.events << ','; writeMap(s.events,inverse);
    s.events << "]}\n" << std::flush;
    ++s.decisions;
    s.pre.erase(it);
  }
  if( controlsQp() )
  {
    Map action = readAction(slice.poc);
    std::lock_guard<std::mutex> lock(s.mutex);
    s.actions.emplace(slice.poc,action);
  }
}

int requestedQp( const Slice& slice, const Area& area, int nativeQpaQp )
{
  const Map& map = session().actions.at(slice.poc);
  const int right = std::min(WIDTH,area.x+int(area.width));
  const int bottom = std::min(HEIGHT,area.y+int(area.height));
  double sum = 0, weight = 0;
  for( int y = area.y/16; y <= (bottom-1)/16; ++y )
    for( int x = area.x/16; x <= (right-1)/16; ++x )
    {
      const int overlap = (std::min(right,(x+1)*16)-std::max(area.x,x*16)) *
                          (std::min(bottom,(y+1)*16)-std::max(area.y,y*16));
      sum += overlap*map[y*COLS+x]; weight += overlap;
    }
  if( weight <= 0 ) throw std::runtime_error("RL-RC-DoT: invalid QG geometry");
  // Delta zero must preserve the encoder's existing native QPA QP.
  const int delta = int(std::lround(sum / weight));
  return Clip3(-slice.sps->qpBDOffset[CH_L],MAX_QP,nativeQpaQp + delta);
}

void recordCu( const CodingUnit& cu )
{
  if( !enabled() || cu.slice->pic->isPreAnalysis || cu.chType != CH_L || !cu.Y().valid() ) return;
  Session& s = session();
  const Area& area = cu.Y();
  const int width = std::min(WIDTH-area.x,int(area.width));
  const int height = std::min(HEIGHT-area.y,int(area.height));
  if( width <= 0 || height <= 0 ) return;
  const double pixels = width*height;
  std::lock_guard<std::mutex> lock(s.mutex);
  FrameStats& f = s.frames[cu.slice->poc];
  f.area += pixels; f.qpSum += pixels*cu.qp;
  if( cu.skip ) f.skip += pixels;
  else if( cu.predMode != MODE_INTER ) f.intra += pixels;
  else f.inter += pixels;
  if( s.cuLog ) s.cuLog << "{\"poc\":" << cu.slice->poc << ",\"x\":" << area.x << ",\"y\":" << area.y
    << ",\"w\":" << width << ",\"h\":" << height << ",\"qp\":" << int(cu.qp) << "}\n";
}

void addSyntaxBits( int poc, bool motion, unsigned bits )
{
  std::lock_guard<std::mutex> lock(session().mutex);
  if( motion ) session().frames[poc].motionBits += bits;
  else session().frames[poc].transformBits += bits;
}

void finishPicture( const Picture& pic )
{
  if( !enabled() || pic.isPreAnalysis ) return;
  Session& s = session();
  const double value = ssim(pic.getOrigBuf(COMP_Y),pic.getRecoBuf(COMP_Y),s.bitDepth);
  if( !std::isfinite(value) || std::isnan(pic.psnr[COMP_Y]) )
    throw std::runtime_error("RL-RC-DoT: nonfinite completed-picture quality");
  const std::string path = std::string(setting("VVENC_RCDOT_RECON_DIR")) + "/" + std::to_string(pic.poc) + ".yuv";
  std::ofstream file(path,std::ios::binary);
  if( !file ) throw std::runtime_error("RL-RC-DoT: cannot write reconstruction");
  for( int comp = 0; comp < 3; ++comp )
  {
    const CPelBuf plane = pic.getRecoBuf(ComponentID(comp));
    for( int y = 0; y < plane.height; ++y )
      for( int x = 0; x < plane.width; ++x )
      {
        const uint16_t sample = uint16_t(plane.at(x,y));
        file.put(char(sample & 255)); file.put(char(sample >> 8));
      }
  }
  file.close();
  if( !file ) throw std::runtime_error("RL-RC-DoT: reconstruction write failed");
  std::lock_guard<std::mutex> lock(s.mutex);
  s.frames[pic.poc].psnr = std::min(99.0,pic.psnr[COMP_Y]);
  s.frames[pic.poc].ssim = value;
}

void outputPicture( const Picture& pic, const AccessUnitList& au )
{
  if( !enabled() || pic.isPreAnalysis ) return;
  uint64_t bytes = 0;
  for( auto it = au.begin(); it != au.end(); ++it )
  {
    const auto type = (*it)->m_nalUnitType;
    const bool four = it == au.begin() || type == VVENC_NAL_UNIT_DCI || type == VVENC_NAL_UNIT_VPS ||
      type == VVENC_NAL_UNIT_SPS || type == VVENC_NAL_UNIT_PPS || type == VVENC_NAL_UNIT_PREFIX_APS || type == VVENC_NAL_UNIT_SUFFIX_APS;
    bytes += (*it)->m_nalUnitData.str().size() + (four ? 4 : 3);
  }
  Session& s = session();
  std::lock_guard<std::mutex> lock(s.mutex);
  FrameStats f = s.frames.at(pic.poc);
  if( int(f.area) != WIDTH*HEIGHT ) throw std::runtime_error("RL-RC-DoT: final luma CU areas do not cover the frame exactly");
  f.bits = bytes*8;
  s.last = f; s.historyPoc = pic.poc; s.bits += f.bits; ++s.outputs;
  s.qpSum += f.qpSum; s.qpArea += f.area;
  s.events << "{\"kind\":\"post\",\"poc\":" << pic.poc << ",\"output_step\":" << s.outputs
    << ",\"au_bytes\":" << bytes << ",\"au_bits\":" << f.bits << ",\"psnr\":" << f.psnr
    << ",\"ssim\":" << f.ssim << ",\"mean_qp\":" << f.qpSum/f.area
    << ",\"cumulative_bits\":" << s.bits << ",\"area\":" << int(f.area)
    << ",\"mode_fractions\":[" << f.intra/f.area << ',' << f.inter/f.area << ',' << f.skip/f.area << ']'
    << ",\"prediction_syntax_bits\":" << f.motionBits << ",\"coefficient_syntax_bits\":" << f.transformBits;
  if( controlsQp() ) { s.events << ",\"consumed_action\":"; writeMap(s.events,s.actions.at(pic.poc)); }
  s.events << "}\n" << std::flush;
  if( s.cuLog ) s.cuLog.flush();
  s.frames.erase(pic.poc); s.actions.erase(pic.poc);
}

Analysis LookAhead::analyze( const CPelBuf& source, const std::vector<Pel>& previous )
{
  const std::vector<Pel> small = lowres(source,m_bitDepth);
  Analysis result;
  constexpr int stride = WIDTH/2;
  for( int by = 0; by < ROWS; ++by )
    for( int bx = 0; bx < COLS; ++bx )
    {
      const int index = by*COLS+bx, x0 = bx*8, y0 = by*8;
      const Pel* block = small.data()+y0*stride+x0;
      double sum = 0, squared = 0;
      Pel predictions[3][64];
      int dc = 0;
      for( int k = 0; k < 8; ++k ) dc += (by ? small[(y0-1)*stride+x0+k] : 128) + (bx ? small[(y0+k)*stride+x0-1] : 128);
      dc = (dc+8)/16;
      for( int y = 0; y < 8; ++y )
        for( int x = 0; x < 8; ++x )
        {
          const int pixel = block[y*stride+x]; sum += pixel; squared += pixel*pixel;
          predictions[0][y*8+x] = Pel(dc);
          predictions[1][y*8+x] = by ? small[(y0-1)*stride+x0+x] : 128;
          predictions[2][y*8+x] = bx ? small[(y0+y)*stride+x0-1] : 128;
        }
      result.energy[index] = std::max(0.0,squared/64-(sum/64)*(sum/64));
      int intra = INT32_MAX;
      for( auto& prediction : predictions ) intra = std::min(intra,satd8(block,stride,prediction,8));
      result.intra[index] = std::max(1,intra);
      int bestX = x0, bestY = y0, sadBest = INT32_MAX;
      if( !previous.empty() )
      {
        for( int dy = -4; dy <= 4; dy += 2 )
          for( int dx = -4; dx <= 4; dx += 2 )
          {
            const int x = Clip3(0,stride-8,x0+dx), y = Clip3(0,HEIGHT/2-8,y0+dy);
            int sad = 0;
            for( int py = 0; py < 8; ++py )
              for( int px = 0; px < 8; ++px ) sad += std::abs(int(block[py*stride+px])-int(previous[(y+py)*stride+x+px]));
            if( sad < sadBest ) { sadBest = sad; bestX = x; bestY = y; }
          }
        result.inter[index] = std::min(result.intra[index],double(satd8(block,stride,previous.data()+bestY*stride+bestX,stride)));
      }
      else result.inter[index] = result.intra[index];
      result.referenceCell[index] = Clip3(0,ROWS-1,(bestY+4)/8)*COLS+Clip3(0,COLS-1,(bestX+4)/8);
    }
  m_previous = small;
  return result;
}

void LookAhead::initPicture( Picture* pic )
{
  const CPelBuf source = pic->getOrigBuf(COMP_Y);
  if( source.width != WIDTH || source.height != HEIGHT ) throw std::runtime_error("RL-RC-DoT: unexpected source geometry");
  m_analysis.emplace(pic->poc,analyze(source,m_previous));
}

void LookAhead::processPictures( const PicList& pictures, AccessUnitList&, PicList& done, PicList& free )
{
  const Picture* first = pictures.front();
  std::vector<int> window;
  for( auto pic : pictures ) if( pic->poc <= first->poc+10 ) window.push_back(pic->poc);
  Map propagated = {};
  for( int i = int(window.size())-1; i > 0; --i )
  {
    Map toReference = {};
    const Analysis& current = m_analysis.at(window[i]);
    for( int cell = 0; cell < CELLS; ++cell )
    {
      const double fraction = (current.intra[cell]-current.inter[cell])/current.intra[cell];
      toReference[current.referenceCell[cell]] += fraction*(current.intra[cell]+propagated[cell]);
    }
    propagated = toReference;
  }
  storeAnalysis(first->poc,window.back(),m_analysis.at(first->poc),propagated);
  m_analysis.erase(first->poc);
  done.push_back(pictures.front()); free.push_back(pictures.front());
}

} // namespace rcdot
} // namespace vvenc
