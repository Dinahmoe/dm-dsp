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


#include "Oscillator.hpp"
#include <cstring>
#include <string> // for memcpy
#include <stdexcept>
#include <cassert>
#include <iostream>
#include "LFOTables.hpp"

#define WAVETABLE_SIZE 1024

#ifdef DM_PLATFORM_CYGWIN
#define M_PI 3.14159265358979323846
#endif

namespace dinahmoe {
namespace dsp {

Oscillator::Oscillator(const float& samplingRate_, const WaveformType& type_, int wavetableSize_) :
  m_samplingRate(samplingRate_),
  m_type(type_),
  m_wavetableSize(WAVETABLE_SIZE + 1),
  m_baseFrequency(m_samplingRate / (m_wavetableSize - 1)),
  m_phase_accumulator(0L) {
  
  if (wavetableSize_ != WAVETABLE_SIZE) {
    throw std::runtime_error("Wavetables can just be 1024 for now");
  }
  
  m_wavetable = new float[m_wavetableSize];
  setType(m_type, nullptr);
}

Oscillator::~Oscillator() {
  delete [] m_wavetable;
}

// http://music.columbia.edu/pipermail/music-dsp/2012-April/070656.html
  
float Oscillator::nextSample(const float& frequency_) {
  float frequencyRatio = frequency_ / m_baseFrequency;
  uint32_t phase_increment = (unsigned long)((float)frequencyRatio * 0x00400000); // 22 bit
  uint32_t wavetable_index= m_phase_accumulator >> 22;
  unsigned long fractional_index = m_phase_accumulator & 0x003FFFFF;
  // the comments refer to an example of the method with 8 bits, wavetable size = 4
  // phaseaccumulator = [0][1][0][0][0][0][1][1] 
  //wavetable_index = m_phase_accumulator >> 22;
  // wavetable_index = [0][0][0][0][0][0][0][1]
  // mask for 8 bits is 0x3F
  // we now want to find the fraction of unity in floating point values
  //fractional_index = m_phase_accumulator & 0x003FFFFF; // these are 22 1s.
  // fractional_index = [0][0][0][0][0][0][1][1]
  // literal is 1 / (2^0x003FFFFF)
  // literal for 8 bits is 1 / 2^6 = 0.01587301587302
  float interp_value = 2.38418579101562e-07 * (float)fractional_index;
  // linear interpolation
  float waveform_out = m_wavetable[wavetable_index++];
  waveform_out += interp_value*(m_wavetable[wavetable_index] - waveform_out);
  m_phase_accumulator += phase_increment;
  return waveform_out;
}

void Oscillator::resetPhase() {
  m_phase_accumulator = 0L;
}


void Oscillator::setPhase(float phase_) {
  assert(phase_ >= 0 && phase_ < 2.0F * M_PI); // normalized please
  m_phase_accumulator = (phase_ / (2.0F * M_PI)) * m_wavetableSize * 0x00400000;
}

void Oscillator::setType(WaveformType type_, const float* table) {
  switch (type_) {
    case SINE:
      SineTable<float>(m_wavetable, m_wavetableSize);
      break;
    case TRIANGLE:
      TriangleTable<float>(m_wavetable, m_wavetableSize);
      break;
    case SAWTOOTH:
      SawtoothTable<float>(m_wavetable, m_wavetableSize);
      break;
    case SQUARE:
      SquareTable<float>(m_wavetable, m_wavetableSize);
      break;
    case CUSTOM:
      memcpy(m_wavetable, table, m_wavetableSize * sizeof(float));
      break;
    default:
      break;
  }
}
  
}}