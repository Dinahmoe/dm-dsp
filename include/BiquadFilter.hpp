/*
 * Copyright (c) 2015, Dinahmoe. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL PETER THORSON BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BiquadFilter_hpp_
#define BiquadFilter_hpp_

#include "Log.h"

#ifdef DM_PLATFORM_CYGWIN
#define M_PI 3.14159265358979323846
#endif

#define KILL_DENORMALS(x) (!(x < -1.0e-8 || x > 1.0e-8)) ? 0 : x

namespace dinahmoe {
namespace DspBasics {

enum BiquadFilterType {
  LOWPASS,
  HIGHPASS,
  BANDPASS,
  LOWSHELF,
  HIGHSHELF,
  PEAKING,
  NOTCH,
  ALLPASS
};

inline BiquadFilterType getFilterTypeFromString(const std::string& filterType_) {
  if (filterType_ == "LOWPASS") {
    return LOWPASS;
  } else if (filterType_ == "HIGHPASS") {
    return HIGHPASS;
  } else if (filterType_ == "BANDPASS") {
    return BANDPASS;
  } else if (filterType_ == "LOWSHELF") {
    return LOWSHELF;
  } else if (filterType_ == "HIGHSHELF") {
    return HIGHSHELF;
  } else if (filterType_ == "PEAKING") {
    return PEAKING;
  } else if (filterType_ == "NOTCH") {
    return NOTCH;
  } else if (filterType_ == "ALLPASS") {
    return ALLPASS;
  } else {
    dmaf_log(dinahmoe::Log::Warning, "Filter type %s not implemented or unknown, defaulting to LOWPASS", filterType_.c_str());
    return LOWPASS;
  }
}


template<typename FTYPE, int MAXCHANNELS>
class BiquadFilter {
public:
  BiquadFilter(float samplingRate, size_t chans_, BiquadFilterType type_, float cutoff_, float Q_, float gain_) :
      m_sampleRate(samplingRate), m_channels(chans_), m_type(type_),
      m_cutoff(cutoff_), m_q(Q_), m_gain(gain_), m_filterResponseHasChanged(true) {
    recomputeCoefficients(m_type, m_cutoff, m_q, m_gain);
    for (int ch = 0; ch < m_channels; ++ch)
      in_1[ch] = in_2[ch] = out_1[ch] = out_2[ch] = 0.0;
  }
  
  void setSamplingRate(float samplingRate_) {
    if (m_sampleRate == samplingRate_) {
      m_sampleRate = samplingRate_;
      recomputeCoefficients();
      for (int ch = 0; ch < MAXCHANNELS; ++ch)
        in_1[ch] = in_2[ch] = out_1[ch] = out_2[ch] = 0.0;
    }
  }
  
  void setChannels(size_t chans_) {
    if (chans_ != m_channels) {
      m_channels = chans_;      
      for (int ch = 0; ch < m_channels; ++ch)
        in_1[ch] = in_2[ch] = out_1[ch] = out_2[ch] = 0.0;
    }
  }
  
  void recomputeCoefficients(BiquadFilterType type_, float cutoff_, float Q_, float gain_);
  
  void process(FTYPE** in_, FTYPE** out, int nSamples_);

  bool filterResponseHasChanged() {
    return m_filterResponseHasChanged;  
  }
  
  float getCutoff() {
    return m_cutoff;
  }
  float getQ() {
    return m_q;
  }
  float getGain() {
    return m_gain;
  }

private:
  void setCoefficients(double& a0_, 
  double& a1_,
  double& a2_,
  double& b0_,
  double& b1_,
  double& b2_) {
    a0 = a0_;
    a1 = a1_ / a0_;;
    a2 = a2_ / a0_;
    b0 = b0_ / a0_;
    b1 = b1_ / a0_;
    b2 = b2_ / a0_;
  }
  void computeLowpassCoefficients();
  void computeHighpassCoefficients();
  void computeBandpassCoefficients();
  void computeLowshelfCoefficients();
  void computeHighshelfCoefficients();
  void computePeakingCoefficients();
  void computeNotchCoefficients();
  void computeAllpassCoefficients();
  
  float m_sampleRate;
  size_t m_channels;
  BiquadFilterType m_type;
  float m_cutoff;
  float m_q;
  float m_gain;
  bool m_filterResponseHasChanged;
  
  double a0;
  double a1;
  double a2;
  double b0;
  double b1;
  double b2;

  FTYPE in_1[MAXCHANNELS], in_2[MAXCHANNELS], out_1[MAXCHANNELS], out_2[MAXCHANNELS];
};


template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::recomputeCoefficients(BiquadFilterType type_, float cutoff_, float Q_, float gain_) {
  m_type = type_;
  m_cutoff = std::max(cutoff_, .0F);
  m_q = std::max(Q_, .0F);
  m_gain = gain_;
  
  switch(m_type) {
    case(LOWPASS):
      computeLowpassCoefficients();
      break;
    case(HIGHPASS):
      computeHighpassCoefficients();
      break;
    case(BANDPASS):
      computeBandpassCoefficients();
      break;
    case(LOWSHELF):
      computeLowshelfCoefficients();
      break;
    case(HIGHSHELF):
      computeHighshelfCoefficients();
      break;
    case(PEAKING):
      computePeakingCoefficients();
      break;
    case(NOTCH):
      computeNotchCoefficients();
      break;
    case(ALLPASS):
      computeAllpassCoefficients();
      break;
    default:
      assert(false);
  };
  m_filterResponseHasChanged = true;
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::process(FTYPE** in_, FTYPE** out_, int nSamples_) {
  int samples = nSamples_;
  for (size_t ch = 0; ch < m_channels; ++ch) {
    FTYPE *currentIn = in_[ch], *currentOut = out_[ch];
    samples = nSamples_;
    // do first two first iterations by hand. 
    // TODO OPTIMIZE this, of course, with SSE, Accelerate and MKH
    while (samples--) {
      *(currentOut) = KILL_DENORMALS(b0 * (*currentIn) + b1 * in_1[ch] + b2 * in_2[ch] -
        a1 * out_1[ch] - a2 * out_2[ch]);
        in_2[ch] = in_1[ch];
        in_1[ch] = *(currentIn++);
        out_2[ch] = out_1[ch];
        out_1[ch] = *(currentOut++); 
    }
  }
}

// all computation and syntax based on the Audio EQ Coockbook by RBJ
// http://www.musicdsp.org/files/Audio-EQ-Cookbook.txt

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeLowpassCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double b0 = (1.0 - cs) / 2.0;
  double b1 =  1.0 - cs;
  double b2 = (1.0 - cs) / 2.0;
  double a0 =  1.0 + AL;
  double a1 = -2.0 * cs;
  double a2 =  1.0 - AL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeHighpassCoefficients() {
  double w0 = 2 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / ( 2 * m_q);
  double b0 =  (1 + cs) / 2;
  double b1 = -(1 + cs);
  double b2 =  (1 + cs) / 2;
  double a0 =  1 + AL;
  double a1 = -2 * cs;
  double a2 =  1 - AL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeBandpassCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double b0 = AL;
  double b1 = .0F;
  double b2 = -AL;
  double a0 =  1.0 + AL;
  double a1 = -2.0 * cs;
  double a2 =  1.0 - AL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeLowshelfCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double A = pow(10.0, m_gain / 40);
  float sqrtAL = 2.F * sqrt(A) * AL;
  double b0 = A *        ((A + 1.F) - ((A - 1.F) * cs) + sqrtAL);
  double b1 = -2.F * A * ((A - 1.F) - (A + 1.F) * cs);
  double b2 = A *        ((A + 1.F) - (A - 1.F) * cs - sqrtAL);
  double a0 =             (A + 1.F) + (A - 1.F) * cs + sqrtAL;
  double a1 = 2.F *      ((A - 1.F) + (A + 1.F) * cs);
  double a2 =             (A + 1.F) + (A - 1.F) * cs - sqrtAL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeHighshelfCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  
  double A = pow(10.0, m_gain / 40);
  float sqrtAL = 2.F * sqrt(A) * AL;
  double b0 = A *        ((A + 1.F) + ((A - 1.F) * cs) + sqrtAL);
  double b1 = -2.F * A * ((A - 1.F) + (A + 1.F) * cs);
  double b2 = A *        ((A + 1.F) + (A - 1.F) * cs - sqrtAL);
  double a0 =             (A + 1.F) - (A - 1.F) * cs + sqrtAL;
  double a1 = 2.F *      ((A - 1.F) - (A + 1.F) * cs);
  double a2 =             (A + 1.F) - (A - 1.F) * cs - sqrtAL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computePeakingCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double A = pow(10.0, m_gain / 40);
  double b0 = 1.F + AL * A;
  double b1 =  -2.F * cs;
  double b2 = 1.F - AL * A;
  double a0 =  1.F + AL / A;
  double a1 = -2.0 * cs;
  double a2 =  1.0 - AL / A;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeNotchCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double b0 = 1.0F;
  double b1 =  -2.0F * cs;
  double b2 = 1.0F;
  double a0 =  1.0 + AL;
  double a1 = -2.0 * cs;
  double a2 =  1.0 - AL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

template<typename FTYPE, int MAXCHANNELS>
void BiquadFilter<FTYPE,MAXCHANNELS>::computeAllpassCoefficients() {
  double w0 = 2.0 * M_PI * m_cutoff / m_sampleRate;
  double cs = cos (w0);
  double sn = sin (w0);
  double AL = sn / (2.0 * m_q);
  double b0 = 1.0 - AL;
  double b1 =  -2.0F * cs;
  double b2 = 1.F + AL;
  double a0 =  1.F + AL;
  double a1 = -2.0 * cs;
  double a2 =  1.0 - AL;
  setCoefficients (a0, a1, a2, b0, b1, b2);
}

}
} // DspBasics

#endif
