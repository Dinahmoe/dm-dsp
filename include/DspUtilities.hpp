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

//  Created by Alessandro Saccoia on 3/19/14.

#ifndef Project_DmDspUtilities_hpp
#define Project_DmDspUtilities_hpp

#include "DmDspConfig.hpp"
#include <cstdlib>

namespace dinahmoe {
namespace dsp {

void dm_stridedMemcpy(float* in_, long inputStride_, float* out_, long outputStride_,unsigned long size_, unsigned long dims_);

void dm_vvadd(const float* input_, float* input2_, float* result_, unsigned long size_);
void dm_vsmul(float value_, float* output_, unsigned long size_);
void dm_vsfill(float value, float * vector_, unsigned long size_);

inline int dm_nextPowerOfTwo(int start_) {
  int toReturn = 1;
  while (toReturn < start_) {
    toReturn <<= 1;
  }
  return toReturn;
}

extern void dm_convolve_td(float *inputData_, unsigned long inputSize_,
  float *impulseData_,  unsigned long impulseSize_,
  float *outputData_,  unsigned long outputSize_);

#define ALLOC_REAL(realVar, size) \
  realVar = (float*)malloc((size) * sizeof(float));

#define DEALLOC_REAL(realVar) \
  free(realVar);
  
#define ZERO_REAL(realVar, offset, length) \
  memset(realVar + offset, 0, length * sizeof(float));

}}

#endif
