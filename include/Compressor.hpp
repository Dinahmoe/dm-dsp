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
 *  Compressor.hpp
 *
 *  Created by Alessandro Saccoia on 3/12/12.
 *  Copyright 2012 Alessandro Saccoia. All rights reserved.
 *
 */

#ifndef __COMPRESSOR_HPP__
#define __COMPRESSOR_HPP__

#include "AudioBuffer.hpp"
#include "BicubicInterpolator.hpp"
#include <cassert>
#include <cmath>

#define KILL_DENORMAL(x) (x < 10e-8) ? 0 : x

namespace dinahmoe {
namespace dsp {

template <class FTYPE>
class Compressor {
public:
  // default ctor needed for arrays
  Compressor() :
    m_attackTime(0.02F), //20 Ms
    m_releaseTime(0.5F), //500 Ms
    m_ratio(4.9f),
    m_kneeLog(5),
    m_thresholdLog(-10),
    m_gain(1),
    m_inputGain(1),
    m_channelLink(0),
    m_autoMakeup(false) {
  
  }
  
  void initialize(FTYPE sr_, int nchan_, int blocksize_);
  
  inline void clear() {
    m_envelopeStorage.zero();
    m_sum.zero();
  }
  
  inline void SetSampleRate(FTYPE sampleRate);
  inline void SetChannels(int channels);
  inline void SetBlockSize(int blocksize);
  
	inline void SetAttackMs(FTYPE fAttackMs);
	inline void SetReleaseMs (FTYPE fReleaseMs);
  inline void SetRatio (FTYPE ratio_);
  inline void SetThresholdDb(FTYPE fThreshold );
	inline void SetThresholdLinear(FTYPE fThreshold );
  inline void SetChannelLink (FTYPE channelLink_); // 0...1
  inline void SetGainDb(FTYPE gain_);
  inline void SetGainLinear(FTYPE gain_);
  inline void SetInputGainDb(FTYPE gain_);
  inline void SetInputGainLinear(FTYPE gain_);
  inline void SetKneeDb(FTYPE knee_);
  inline void SetAutomakeUp(bool automakeup_);

  void FeedDriveSignal(const AudioBufferC<FTYPE>& signal, AudioBufferC<FTYPE>& envelope);
  void FeedZeroSignal(); // this zeroes the envelope storage!
  void Process(const AudioBufferC<FTYPE>& signal, AudioBufferC<FTYPE>& output, const AudioBufferC<FTYPE>& sidechain);
  
private:

  // computes the coefficient for a moving average one-pole 
  // filter that reaches the 1 - (1/e) ~ 63% of its final  
  // value in (time_constant * fs) sec. This could be the place
  // to do the distinction with the optoo compressor
  inline FTYPE OnePoleCoeff(FTYPE time_constant);
  
  inline void UpdateBicubicInterpolator() {
    m_interpolator.setCoefficients(m_kneeLog,
      m_kneeLog/FTYPE(2) + (m_kneeLog/FTYPE(2))/m_ratio,
      FTYPE(1) / m_ratio);
  }
  
  inline void UpdateMakeupGain();
  
  FTYPE	m_sampleRate;
  int m_channels;
  int m_blocksize;
  //keeps y[n-1] for every channel
  AudioBufferC<FTYPE> m_envelopeStorage;
  //keeps the 100% linked envelope
  AudioBufferC<FTYPE> m_sum;
  
	FTYPE	m_attackTime;
	FTYPE	m_releaseTime;
	FTYPE	m_attackFactor;
	FTYPE	m_releaseFactor;
  FTYPE	m_slope;
  FTYPE	m_ratio;
  FTYPE m_kneeLog;
  FTYPE	m_threshold;
	FTYPE	m_thresholdLog;
	FTYPE	m_gain;
  FTYPE m_inputGain;
  FTYPE m_channelLink;
  FTYPE m_channelLinkNeg;

  FTYPE m_kneeBeginLog;
  FTYPE m_kneeEndLog;
  bool m_autoMakeup;
  BicubicInterpolator<FTYPE> m_interpolator;
};

template <class FTYPE>
inline void Compressor<FTYPE>::initialize(FTYPE sr_, int nchan_, int blocksize_) {
  SetSampleRate(sr_);
  SetChannels(nchan_);
  SetBlockSize(blocksize_);
  
  SetRatio(m_ratio);
  SetThresholdDb(m_thresholdLog);
  SetKneeDb(m_kneeLog);
  SetChannelLink(m_channelLink);
  SetAutomakeUp(m_autoMakeup);
  
  for (int i = 0; i < m_channels; ++i)
	m_envelopeStorage.data[i][0] = .0F;
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetSampleRate(FTYPE sampleRate) {
  m_sampleRate = sampleRate;
  SetAttackMs(m_attackTime);
  SetReleaseMs(m_releaseTime);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetChannels(int channels) {
  m_channels = channels;
  m_envelopeStorage.resize(m_channels, 1);
  m_envelopeStorage.zero();
  for (int i = 0; i < m_channels; ++i)
	m_envelopeStorage.data[i][0] = .0F;
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetBlockSize(int blocksize) {
  m_blocksize = blocksize;
  m_sum.resize(1, m_blocksize);
  
  for (int i = 0; i < m_channels; ++i)
	m_envelopeStorage.data[i][0] = .0F;
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetAttackMs(FTYPE fAttackMs) {
  m_attackTime = fAttackMs/FTYPE(1000);
  m_attackFactor = OnePoleCoeff(m_attackTime);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetReleaseMs(FTYPE fReleaseMs) {
  m_releaseTime = fReleaseMs/FTYPE(1000);
  m_releaseFactor = OnePoleCoeff(m_releaseTime);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetRatio (FTYPE ratio_) {
  m_ratio = ratio_;
  m_slope = FTYPE(1) - FTYPE(1) / m_ratio;
  m_slope /= 20;
  UpdateBicubicInterpolator();
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetThresholdDb(FTYPE fThreshold ) {
  m_thresholdLog = fThreshold;
  m_threshold = pow(FTYPE(10), fThreshold / FTYPE(20));
  SetKneeDb(m_kneeLog);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetThresholdLinear(FTYPE fThreshold ) {
  m_threshold = fThreshold;
  m_thresholdLog = FTYPE(20) * log10(fThreshold);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetAutomakeUp(bool automakeup_) {
  m_autoMakeup = automakeup_;
  if (m_autoMakeup) {
    UpdateMakeupGain();
  }
}


// the magic number is based on an experimental, subjective measurement
// on the perceived loudness..
// will need to decide wether to keep this or not
// see http://music.columbia.edu/pipermail/music-dsp/2009-September/068030.html
template <class FTYPE>
inline void Compressor<FTYPE>::UpdateMakeupGain() {
  const FTYPE magicCoefficient = 3.3f;
  FTYPE makeUpDb = -(m_thresholdLog - (m_thresholdLog / m_ratio)) / magicCoefficient;
  m_gain = pow(FTYPE(10), makeUpDb / FTYPE(20));
}


// NOTE: I disable the makeup setting if automakeup is on... should be done fromg the GUI instead?
template <class FTYPE>
inline void Compressor<FTYPE>::SetGainDb(FTYPE gain_) {
  if (!m_autoMakeup) {
    m_gain = pow(FTYPE(10), gain_ / FTYPE(20));
  }
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetGainLinear(FTYPE gain_) {
  if (!m_autoMakeup) {
    m_gain = gain_;
  }
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetInputGainDb(FTYPE gain_) {
  m_inputGain = pow(FTYPE(10), gain_ / FTYPE(20));
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetInputGainLinear(FTYPE gain_) {
  m_inputGain = gain_;
}


template <class FTYPE>
inline void Compressor<FTYPE>::SetChannelLink (FTYPE channelLink_) {
  m_channelLink = channelLink_;
  m_channelLinkNeg = FTYPE(1 - channelLink_);
}

template <class FTYPE>
inline void Compressor<FTYPE>::SetKneeDb(FTYPE knee_) {
  m_kneeLog = knee_;
  m_kneeBeginLog = m_thresholdLog - m_kneeLog / FTYPE(2);
  m_kneeEndLog = m_thresholdLog + m_kneeLog / FTYPE(2);
  UpdateBicubicInterpolator();
}

template <class FTYPE>
inline FTYPE Compressor<FTYPE>::OnePoleCoeff(FTYPE time_constant) {
  if(time_constant > 0.0) {
		return FTYPE(1) - (exp(FTYPE(-1) / (time_constant * m_sampleRate)));
	}
	return FTYPE(0);
}


// TODO: opto characteristic... instead of a normal exponential curve
// design a curve that whose attack and release coefficients depend on the current value of the
// envelope


// NOTE: this computes the envelope in LINEAR domain, as the gain computer expects, so 
// for sidechain configuration make sure to use this method, or another one that provides
// linear values
template <class FTYPE>
void Compressor<FTYPE>::FeedDriveSignal(const AudioBufferC<FTYPE>& signal, AudioBufferC<FTYPE>& envelope) {
  int nsamples = signal.size;
  FTYPE samplein;
  FTYPE *in;
  FTYPE *env;
  FTYPE *storage;
  m_sum.zero();
  FTYPE *sum = m_sum.data[0];
  
  // compute the envelope for every channel, independently  
  // Smooth branching peak detector
  for (int chan = 0; chan < m_channels; ++chan) {
    in = signal.data[chan];
    env = envelope.data[chan];
    storage = m_envelopeStorage.data[chan];
    sum = m_sum.data[0];
    for (int sample = 0; sample < nsamples; ++sample) {
      samplein = fabs(*in++);
      // TODO: kill denormals by setting very low numbers in this subtraction to zero
      if (samplein > *storage) {
        *env = *storage + (samplein - *storage) * m_attackFactor;
      } else {
        *env = *storage + (samplein - *storage) * m_releaseFactor;
      }   
      *env = KILL_DENORMAL(*env);   
      // store the 100% linked envelope in sum, avoid branching on purpose
      if (*env > *sum) {
        *sum = *env;
      }
      ++sum;
      *storage = *env++;
    }
  }
  if (m_channelLink > FTYPE(0)) {
    for (int chan = 0; chan < m_channels; ++chan) {
      env = envelope.data[chan];
      sum = m_sum.data[0];
      for (int sample = 0; sample < nsamples; ++sample) {
        *env = *env + (*sum - *env) * m_channelLink;
        ++env, ++sum;
      }    
    }  
  }
}

template <class FTYPE>
void Compressor<FTYPE>::FeedZeroSignal() {
  m_envelopeStorage.zero();
}



// this method applies the gain envelope, can work in-place.
// if the sidechain comes from another detector, make sure that it contains
// the same number of channels as the input, and it's in linear scale
template <class FTYPE>
void Compressor<FTYPE>::Process(const AudioBufferC<FTYPE>& signal, AudioBufferC<FTYPE>& output, const AudioBufferC<FTYPE>& envelope) {
//  assert(signal.channels == channels && sidechain.channels == channels &&
//         signal.count <= blocksize && sidechain.count <= blocksize);
  int nsamples = signal.size;
  FTYPE *in;
  FTYPE *out;
  FTYPE *envelopeIn;
  FTYPE logEnvelope;
  for (int chan = 0; chan < m_channels; ++chan) {
    in = signal.data[chan];
    out = output.data[chan];
    envelopeIn = envelope.data[chan];
    for (int sample = 0; sample < nsamples; ++sample) {
      // do this computations in the log domain because the value in the
      // knee region is the solution of a 3rd degree polynomial in log domain, 
      // but it's a highr degree in linear domain
      logEnvelope = (*envelopeIn < 1e-6) ? FTYPE(-120) : FTYPE(20) * log10(*envelopeIn);
      if (logEnvelope > m_kneeBeginLog) {        
        if (logEnvelope < m_kneeEndLog) {
          // subtract the interpolated value from the 
          FTYPE gainReduction = (logEnvelope - m_kneeBeginLog) - m_interpolator.interpolate(logEnvelope - m_kneeBeginLog);
          gainReduction = pow(FTYPE(10), ((-gainReduction) / FTYPE(20))); 
          *out = *in * gainReduction;
        } else { 
          *out = *in * pow(FTYPE(10), (m_thresholdLog - logEnvelope) * m_slope); 
        }
      } else {
        *out = *in;
      }
      // makeup gain
      *out *= m_gain;
      ++in, ++envelopeIn, ++out;
    }
  }

}

}}

#endif // __COMPRESSOR_HPP__
