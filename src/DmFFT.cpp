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
//  Created by Alessandro Saccoia on 3/13/14.


#include "DmFFT.hpp"

#ifdef DSP_USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#else
#include "kiss_fftr.h"
#endif

#include "DspUtilities.hpp"
#include "DmDspConfig.hpp"


namespace dinahmoe {
namespace dsp {

namespace {
#ifdef DSP_USE_ACCELERATE
  struct fftsetup_internal {
    FFTSetup fftSetup;
    vDSP_Length logFftSize;
    dm_fftDirection direction;
  };
#else
  struct fftsetup_internal {
    kiss_fftr_cfg fftSetup;
    unsigned long fftSize;
    dm_fftDirection direction;
    float* temp;
  };
#endif
}

dm_fftSetup dm_fftSetupCreate(unsigned long size_, dm_fftDirection direction_) {
  fftsetup_internal* toReturn = new fftsetup_internal();
  toReturn->direction = direction_;
#ifdef DSP_USE_ACCELERATE
  toReturn->logFftSize = log2f(size_);
  toReturn->fftSetup = vDSP_create_fftsetup(toReturn->logFftSize, FFT_RADIX2);
#else
  toReturn->fftSize = size_;
  toReturn->fftSetup = kiss_fftr_alloc(size_, (toReturn->direction == DM_FFT_FORWARD) ? 0 : 1, 0, 0);
  toReturn->temp = new float[size_];
  memset(toReturn->temp, 0, sizeof(float) * size_);
#endif
  return toReturn;
}

float dm_fftGetNormalizationFactor(unsigned int fftSize_) {
#ifdef DSP_USE_ACCELERATE
  return (float) 1.0 / (4 * fftSize_);
#else
  return (float) 1.0 / fftSize_;
#endif
}

void dm_fftSetupExecute(dm_fftSetup fftSetup_, unsigned long timeDomainLength_, float* timeDomain_, dm_fftComplexArrayT*  frequency_, unsigned long offset_) {
  fftsetup_internal* setup = (fftsetup_internal*)fftSetup_;
#ifdef DSP_USE_ACCELERATE
  if (setup->direction == DM_FFT_FORWARD) {
    frequency_->fillFromVector(timeDomain_, timeDomainLength_/2);
    vDSP_fft_zrip(setup->fftSetup, (DSPSplitComplex*)frequency_, 1, setup->logFftSize, FFT_FORWARD);
  } else {
    vDSP_fft_zrip(setup->fftSetup, (DSPSplitComplex*)frequency_, 1, setup->logFftSize, FFT_INVERSE);
    frequency_->toRealVector(timeDomain_, offset_, timeDomainLength_);
  }
#else
  
  if (setup->direction == DM_FFT_FORWARD) {
    memcpy(setup->temp, timeDomain_, sizeof(float) * timeDomainLength_);
    kiss_fftr(setup->fftSetup, (const kiss_fft_scalar *)setup->temp, (kiss_fft_cpx*)frequency_->data);
  } else {
    kiss_fftri(setup->fftSetup, (const kiss_fft_cpx*)frequency_->data, (kiss_fft_scalar *)setup->temp);
    memcpy(timeDomain_, setup->temp + offset_ * 2, timeDomainLength_ * sizeof(float));
    dm_vsfill(0, timeDomain_ + timeDomainLength_, frequency_->size());
  }
  
#endif
}

void dm_fftSetupDestroy(dm_fftSetup fftSetup_) {
  fftsetup_internal* setup = (fftsetup_internal*)fftSetup_;
#ifdef DSP_USE_ACCELERATE
  vDSP_destroy_fftsetup(setup->fftSetup);
#else
  free(setup->fftSetup);
  delete setup->temp;
#endif  
  delete (fftsetup_internal*)fftSetup_;
}

}}

