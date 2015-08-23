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

#include "DspUtilities.hpp"
#include <algorithm>
#include <functional>

#ifdef DSP_USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif



namespace dinahmoe {
namespace dsp {

void dm_stridedMemcpy(float* in_, long inputStride_, float* out_, long outputStride_,unsigned long size_, unsigned long dims_) {
  for (unsigned long dim = 0; dim < dims_; ++dim) {
    for (unsigned long i = dim, o = dim; o < size_; i+=inputStride_, o+=outputStride_) {
      out_[o] = *(in_ + i);
    }
  }
}

void dm_vvadd(const float* input_, float* input2_, float* result_, unsigned long size_) {
#ifdef DSP_USE_ACCELERATE
  vDSP_vadd(input_, 1, input2_, 1, result_, 1, size_);
#else // generic
  std::transform(input_, input_ + size_, input2_, result_, std::plus<float>());
#endif

}

void dm_vsmul(float value_, float* output_, unsigned long size_) {
#ifdef DSP_USE_ACCELERATE
  vDSP_vsmul(output_, 1, &value_, output_, 1, size_);
#else // generic
  std::transform(output_, output_ + size_, output_, std::bind2nd(std::multiplies<float>(), value_));
#endif
}

void dm_vsfill(float value, float * vector_, unsigned long size_) {
#ifdef DSP_USE_ACCELERATE
  vDSP_vfill(&value, vector_, 1,size_);
#else // generic
  std::fill(vector_, vector_ + size_, value);
#endif
}

void dm_convolve_td(float *inputData_, unsigned long inputSize_,
  float *impulseData_,  unsigned long impulseSize_,
  float *outputData_,  unsigned long outputSize_) {
  unsigned long k = 0, o = 0;
  int i = 0;
  for (o = 0; o < outputSize_; ++o) {
    outputData_[o] = .0F;
    for (k = 0; k < impulseSize_; ++k) {
      i = (int)o - (int)k;
      if (i < 0)
        break;
      if (i > (long)inputSize_) {
        continue;
      }
      outputData_[o] += inputData_[i] * impulseData_[k];
    }
  }
}
  

}}
