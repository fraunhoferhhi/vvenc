// Retired DOT_PROXY interface. Use EncoderLib/RCDot.h (adapter/2).
#pragma once
#include "CommonLib/Picture.h"
#include "RateCtrl.h"
#include "vvenc/vvencCfg.h"
#include <stdexcept>
namespace vvenc { namespace rl {
class DotObservation {
public:
  static void emit(const Slice&,const VVEncCfg&,const RateCtrl*) {
    throw std::runtime_error("DOT_PROXY retired; use VVENC_RCDOT_MODE and scripts.rl_rc_dot.session");
  }
};
}}
