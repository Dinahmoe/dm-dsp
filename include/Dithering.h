/*
 * Copyright (c) 2012, Alessandro Saccoia. All rights reserved.
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
 /*
 *  Dithering.h
 *
 *  Created by Alessandro Saccoia on 6/9/12.
 *
 */

#ifndef __DITHERING_H__
#define __DITHERING_H__

#include "AudioBuffer.h"
#include <cmath>

enum DitherType {
  kDitherTriangular,
  kDitherRectangular,
  kDitherTriangularHp
};

// Base class for dithering
template <class FTYPE>
class DitheringStrategy {  
public:

  virtual ~DitheringStrategy(){}
  
  virtual void setTargetQuantization(int quant_) {
    m_targetQuantization = quant_;
    m_fullRange = 1.0f / pow(2, DitheringStrategy<FTYPE>::m_targetQuantization);
    m_halfRange = m_fullRange / 2.0f;
  }
  
  virtual FTYPE operator() (FTYPE sample) = 0;
  
protected:
  int m_targetQuantization;
  FTYPE m_fullRange;
  FTYPE m_halfRange;
};


//----------- Triangular PDF dithering ----------

template <class FTYPE>
class TriangleDither : public DitheringStrategy<FTYPE> {  
public:  
  virtual inline FTYPE operator() (FTYPE sample) {
      /* verbose version... all the next ones are rearrangement for performance
      float nRand = (rand() / (float)RAND_MAX - 0.5f) * (1.F / (pow(2, (float)DitheringStrategy<FTYPE>::m_targetQuantization)));
      sample += nRand + (rand() / (float)RAND_MAX - 0.5f) * (1.F / (pow(2, (float)DitheringStrategy<FTYPE>::m_targetQuantization)));
      short version follows */
      sample += (rand() / (FTYPE)RAND_MAX) * DitheringStrategy<FTYPE>::m_fullRange;
      sample += (rand() / (FTYPE)RAND_MAX) * DitheringStrategy<FTYPE>::m_fullRange;
      sample -= DitheringStrategy<FTYPE>::m_fullRange;
      return sample;
  }
};

//----------- Rectangular PDF dithering ----------

template <class FTYPE>
class RectangleDither : public DitheringStrategy<FTYPE> {  
public:  
  virtual inline  FTYPE operator() (FTYPE sample) {
    sample += (rand() / (FTYPE)RAND_MAX) * DitheringStrategy<FTYPE>::m_fullRange - DitheringStrategy<FTYPE>::m_halfRange;
    //sample += (rand() / (float)RAND_MAX - 0.5f) * (1.F / (pow(2, DitheringStrategy<FTYPE>::m_targetQuantization)));
    return sample;
  }
  
  
};

//----------- Triangular PDF highpass dithering ----------

template <class FTYPE>
class TriangleHpDither : public DitheringStrategy<FTYPE> {  
public:  
  virtual inline FTYPE operator() (FTYPE sample) {
      sample += (rand() / (FTYPE)RAND_MAX) * DitheringStrategy<FTYPE>::m_fullRange;
      sample += (rand() / (FTYPE)RAND_MAX) * DitheringStrategy<FTYPE>::m_fullRange;
      sample -= DitheringStrategy<FTYPE>::m_fullRange;
      sample -= m_dn_1;
      m_dn_1 = sample;
      return sample;
  }
private:
  FTYPE m_dn_1;
};

template <class FTYPE>
class Dithering {
public:
	Dithering() { }
  
  void initialize(FTYPE sr_, int nchan_); 
  
  inline void SetSampleRate(FTYPE sr_) {
    m_sampleRate = sr_;
  }
  
  inline void SetChannels(int nchan_) {
    m_channels = nchan_;
  }
  
  inline void SetType(DitherType type_) {
    m_type = type_;
    switch (m_type) {
      case kDitherTriangular:
        mp_currentStrategy = mp_triangleStrategy;
        break;
      case kDitherRectangular:
        mp_currentStrategy = mp_rectangleStrategy;
        break;
      case kDitherTriangularHp:
        mp_currentStrategy = mp_triangleHpStrategy;
        break;
      default:
        assert(false);
        break;
    }
  }
  
  void setTargetQuantization(int quant_) {
    m_targetQuantization = quant_;
    mp_currentStrategy->setTargetQuantization(m_targetQuantization);
  }
  
  // this can be inplace
  void Process(AudioBuffer<FTYPE>& signal, AudioBuffer<FTYPE>& output, const int nsamples);
private:
  FTYPE	m_sampleRate;
  int m_channels;
  
	DitherType m_type;
  int m_targetQuantization;
  DitheringStrategy<FTYPE>* mp_currentStrategy;
  TriangleDither<FTYPE>* mp_triangleStrategy;
  RectangleDither<FTYPE>* mp_rectangleStrategy;
  TriangleHpDither<FTYPE>* mp_triangleHpStrategy;
};

template <class FTYPE>
void Dithering<FTYPE>::initialize(FTYPE sr_, int nchan_) {
  m_sampleRate = sr_;
  m_channels = nchan_;
  m_targetQuantization = 16;
  
  mp_triangleStrategy = new TriangleDither<FTYPE>();  
  mp_triangleStrategy->setTargetQuantization(m_targetQuantization);
  
  mp_rectangleStrategy = new RectangleDither<FTYPE>();  
  mp_rectangleStrategy->setTargetQuantization(m_targetQuantization);
  
  mp_triangleHpStrategy = new TriangleHpDither<FTYPE>();  
  mp_triangleHpStrategy->setTargetQuantization(m_targetQuantization);
  
  mp_currentStrategy = mp_triangleStrategy;  
}

template <class FTYPE>
void Dithering<FTYPE>::Process(AudioBuffer<FTYPE>& signal, AudioBuffer<FTYPE>& output, const int nsamples) {
  FTYPE *inPtr, *outPtr;
  for (int chan = 0; chan < m_channels; ++chan) {
    inPtr = signal.data[chan];
    outPtr = output.data[chan];
    std::transform(inPtr, inPtr + nsamples, outPtr, std::bind1st(std::mem_fun(&DitheringStrategy<FTYPE>::operator()), mp_currentStrategy));
  }
}

#endif // __DITHERING_H__
