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

#ifndef __Project__DmFFT__
#define __Project__DmFFT__

#include "DmComplex.hpp"
#include "DmDspConfig.hpp"

namespace dinahmoe {
namespace dsp {

typedef void* DmOpaquePtr;
typedef DmOpaquePtr dm_fftSetup;

#ifdef DSP_USE_ACCELERATE
typedef struct DmComplexArraySplit dm_fftComplexArrayT;
#else
typedef struct DmComplexArray dm_fftComplexArrayT;
#endif

enum dm_fftDirection { DM_FFT_FORWARD, DM_FFT_INVERSE };

extern dm_fftSetup dm_fftSetupCreate(unsigned long size_, dm_fftDirection direction);

extern float dm_fftGetNormalizationFactor(unsigned int fftSize_);
/**
 @brief Multi-platform real FFT routine
 
 @param fftSetup_ The setup of the FFT
 @param timeDomainLength_ Length of the samples in/to the time domain buffer
 @param timeDomain_ Array of samples that will be read. shall have at least timeDomainLength_ allocated
 @param frequency_ Array of complex numbers, shall have timeDomainLength_/2 samples allocated
 @param offset_ Defaults to zero, specifies from where in the frequency domain the samples will be copied.
                Valid just for INVERSE_FFT
 */
void dm_fftSetupExecute(dm_fftSetup fftSetup_, unsigned long timeDomainLength_,
  float* timeDomain_, dm_fftComplexArrayT*  frequency_, unsigned long offset = 0);
  
  
extern void dm_fftSetupDestroy(dm_fftSetup fftSetup_);

}}


#endif /* defined(__Project__DmFFT__) */
