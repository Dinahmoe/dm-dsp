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

#include "DmDspConfig.hpp"
#include "DmComplex.hpp"
#include <cassert>
#include <string>
#include <string.h>

#ifdef DSP_USE_ACCELERATE
#include <Accelerate/Accelerate.h>
#endif

#include "DspUtilities.hpp"

namespace dinahmoe {
namespace dsp {

DmComplexArraySplit::DmComplexArraySplit() :
  realp(nullptr), imagp(nullptr), m_size(0) {
}

DmComplexArraySplit::DmComplexArraySplit(const unsigned long size_) :
  realp(nullptr), imagp(nullptr), m_size(size_) {
  if (size_) {
    realp = new float[size_];
    imagp = new float[size_];
  }
}

DmComplexArraySplit::~DmComplexArraySplit() {
  delete [] realp;
  delete [] imagp;
}

void DmComplexArraySplit::resize(const unsigned long size_) {
  delete [] realp;
  delete [] imagp;
  realp = nullptr;
  imagp = nullptr;
  if (size_) {
    realp = new float[size_];
    imagp = new float[size_];
  }
  m_size = size_;
}

void DmComplexArraySplit::fillFromVector(float* inputData_, unsigned long size_) {
  assert(realp && imagp);
  dm_ctoz((const DmComplex*)inputData_, this, size_);
}

void DmComplexArraySplit::zero() {
  assert(realp && imagp);
  dm_vsfill(0.F, realp, m_size);
  dm_vsfill(0.F, imagp, m_size);
}

void DmComplexArraySplit::toRealVector(float* outputData_, unsigned long offset_, unsigned long outputCount_) {
  DmComplexArraySplit temp(0);
  temp.realp = this->realp + offset_;
  temp.imagp = this->imagp + offset_;
  dm_ztoc(&temp, (DmComplex *)outputData_, outputCount_);
  temp.realp = nullptr;
  temp.imagp = nullptr;
}

// ************************************************************
// BEGIN DmCOmplexArray implementation
// ************************************************************

DmComplexArray::DmComplexArray() :
  data(nullptr), m_size(0) {
}

DmComplexArray::DmComplexArray(const unsigned long size_) :
  data(nullptr), m_size(size_) {
  if (size_) {
    data = new DmComplex[size_ + 1];
  }
}

DmComplexArray::~DmComplexArray() {
  delete [] data;
}

void DmComplexArray::resize(const unsigned long size_) {
  delete [] data;
  data = nullptr;
  if (size_) {
    data = new DmComplex[size_ + 1];
  }
  m_size = size_;
}

void DmComplexArray::fillFromVector(float* inputData_, unsigned long size_) {
  assert(data);
  memcpy(this->data, inputData_, sizeof(float) * size_);
  data[m_size].realp = .0F;
  data[m_size].imagp = .0F;
}

void DmComplexArray::zero() {
  dm_vsfill(0.F, (float*)this->data, (m_size + 1) * 2);
}

void DmComplexArray::toRealVector(float* outputData_, unsigned long offset_, unsigned long outputCount_) {
  DmComplexArraySplit temp(0);
  memcpy(outputData_, this->data + (offset_*2), outputCount_);
}



// ************************************************************
// Utility functions implementation
// ************************************************************

void dm_ctoz(
  const DmComplex input_[],
  DmComplexArraySplit * output_,
  unsigned long size_) // always the number of complex numbers, of input elements
{
#ifdef DSP_USE_ACCELERATE
  vDSP_ctoz((DSPComplex*)input_, vDSP_Stride(2), (DSPSplitComplex*)output_, vDSP_Stride(1), vDSP_Length(size_));
#else
  dm_stridedMemcpy((float*)input_, 1,(float*)output_, 2, size_, 2);
#endif
}

void 
dm_ztoc(
  const DmComplexArraySplit * input_,
  DmComplex * output_,
  unsigned long size_) // always the number of complex numbers, of output elements
{
#ifdef DSP_USE_ACCELERATE
  vDSP_ztoc((DSPSplitComplex*)input_, vDSP_Stride(1), (DSPComplex*)output_, vDSP_Stride(2), vDSP_Length(size_/2));
#else
  dm_stridedMemcpy((float*)input_, 2,(float*)output_, 1, size_, 2);
#endif

}

void dm_cvMultiply(
  const DmComplexArray * in1,
  const DmComplexArray * in2,
  const DmComplexArray * out,
  int size) {
  float* in1dataR = (float*)in1->data;
  float* in2dataR = (float*)in2->data;
  float* in1dataI = (float*)in1->data+1;
  float* in2dataI = (float*)in2->data+1;
  float* outdata = (float*)out->data;
  float* outdataEnd = (float*)(out->data) + ((size + 1) * 2);
  int i = 0;
  while (outdata != outdataEnd) {
    *(outdata++) = ((*in1dataR) * (*in2dataR)) - ((*in1dataI) * (*in2dataI));
    *(outdata++) = ((*in1dataR) * (*in2dataI)) + ((*in1dataI) * (*in2dataR));
    in1dataR+=2;
    in2dataR+=2;
    in1dataI+=2;
    in2dataI+=2;
    ++i;
  }
}

// https://developer.apple.com/library/mac/documentation/Accelerate/Reference/vDSPRef/Art/vdsp_41.gif
void dm_cvMultiply(
  const DmComplexArraySplit * in1,
  const DmComplexArraySplit * in2,
  const DmComplexArraySplit * out,
  int size) {
#ifdef DSP_USE_ACCELERATE
  float preserveSigNyq = in1->imagp[0];
  in1->imagp[0] = 0;
  float preserveIRNyq = in2->imagp[0];
  in2->imagp[0] = 0;
  vDSP_zvmul((const DSPSplitComplex *)in1, 1, (const DSPSplitComplex *)in2, 1, (const DSPSplitComplex *)out, 1, size, 1);
  out->imagp[0] = preserveIRNyq * preserveSigNyq;
  in1->imagp[0] = preserveSigNyq;
  in2->imagp[0] = preserveIRNyq;
#else
  for (unsigned int i = 0; i < out->size(); ++i) {
    *(out->realp + i) = ((*(in1->realp)) * (*(in2->realp))) - ((*(in1->imagp)) * (*(in2->imagp)));
    *(out->imagp + i) = ((*(in1->realp)) * (*(in2->imagp))) + ((*(in1->imagp)) * (*(in2->realp)));
  }
#endif
}

void dm_cvAdd(
  const DmComplexArray * in1,
  const DmComplexArray * in2,
  const DmComplexArray * out,
  int size) {
  for (int i = 0; i < (size+1); ++i) {
    out->data[i].realp = in1->data[i].realp + in2->data[i].realp;
    out->data[i].imagp = in1->data[i].imagp + in2->data[i].imagp;
  }
}

// https://developer.apple.com/library/mac/documentation/Accelerate/Reference/vDSPRef/Art/vdsp_41.gif
void dm_cvAdd(
  const DmComplexArraySplit * in1,
  const DmComplexArraySplit * in2,
  const DmComplexArraySplit * out,
  int size) {
#ifdef DSP_USE_ACCELERATE
  vDSP_zvadd((DSPSplitComplex *)in1, 1, (DSPSplitComplex *)in2, 1, (DSPSplitComplex *)out, 1, size);
#else
  dm_vvadd(in1->realp, in2->realp, out->realp, size);
  dm_vvadd(in1->imagp, in2->imagp, out->imagp, size);
#endif
}



}}