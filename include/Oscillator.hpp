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

//  Created by Alessandro Saccoia on 2/26/14.

#ifndef __Dmaf_Offline_Audio_Renderer__Oscillator__
#define __Dmaf_Offline_Audio_Renderer__Oscillator__

#include <cstring>
#include <string> // for memcpy
#include <stdexcept>

namespace dinahmoe {
namespace dsp {

struct Oscillator {
  enum WaveformType {
    SINE,
    TRIANGLE,
    SAWTOOTH,
    SQUARE,
    CUSTOM
  };
  
  Oscillator(const float& samplingRate_, const WaveformType& type_, int wavetableSize_ = 1024);
  ~Oscillator();
  float nextSample(const float& frequency_);
  void resetPhase();
  void setPhase(float phase_);
  void setType(WaveformType type_, const float* table);
private:  
  Oscillator();
  float m_samplingRate;
  WaveformType m_type;
  float* m_wavetable;
  int m_wavetableSize;
  unsigned int m_phase;
  
  float m_baseFrequency;
  uint32_t m_phase_accumulator;
};

}}

#endif /* defined(__Dmaf_Offline_Audio_Renderer__Oscillator__) */
