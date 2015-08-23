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
 
//  Created by Alessandro Saccoia on 3/12/14.
//
//
// The structures and methods have a one to one correspondence with
// the ones in the Apple Accelerate Framework

#ifndef _DmComplex_hpp_
#define _DmComplex_hpp_

namespace dinahmoe {
namespace dsp {

//equivalent of DSPComplex
struct DmComplex {
  float realp;
  float imagp;
};


// equivalent of Accelerate DSPSplitComplex
struct DmComplexArraySplit {
  DmComplexArraySplit();
  DmComplexArraySplit(const unsigned long size_);
  ~DmComplexArraySplit();
  void resize(const unsigned long size_);
  unsigned long size() const { return m_size; }
  void fillFromVector(float* inputData_, unsigned long size_); // no check on the max size
  void toRealVector(float* outputData_, unsigned long offset_, unsigned long outputCount_);
  void zero();
  // note that these should be the first members,
  // because in some methods we take the address of the struct directly ;-)
  float * realp;
  float * imagp;
private:
  // disable copy operator
  DmComplexArraySplit(const DmComplexArraySplit& rhs);
  DmComplexArraySplit& operator=(const DmComplexArraySplit& rhs);
  unsigned long m_size;
};

// compatible with Accelerate DSPComplex[]
// compatible with KissFFT kiss_fft_cpx
struct DmComplexArray {
  DmComplexArray();
  DmComplexArray(const unsigned long size_);
  ~DmComplexArray();
  void resize(const unsigned long size_);
  unsigned long size() const { return m_size; }
  void fillFromVector(float* inputData_, unsigned long size_); // no check on the max size
  void toRealVector(float* outputData_, unsigned long offset_, unsigned long outputCount_);
  void zero();
  DmComplex* data; // taking the address of the structure returns this address.
private:
  // disable copy operator
  DmComplexArray(const DmComplexArray& rhs);
  DmComplexArray& operator=(const DmComplexArray& rhs);
  unsigned long m_size;
};


/*  ctoz and ctozD convert a complex array to a complex-split array.
    ztoc and ztocD convert a complex-split array to a complex array.
*/
extern void dm_ctoz(
  const DmComplex* input_,
  DmComplexArraySplit * output_,
  unsigned long size_); // this is just the number of complex numbers

extern void dm_ztoc(
  const DmComplexArraySplit * input_,
  DmComplex* output_,
  unsigned long outputSamples_); // this is just the number of complex numbers

extern void dm_cvMultiply(
  const DmComplexArray * in1,
  const DmComplexArray * in2,
  const DmComplexArray * out,
  int size);
  
extern void dm_cvMultiply(
  const DmComplexArraySplit * in1,
  const DmComplexArraySplit * in2,
  const DmComplexArraySplit * out,
  int size);
  
extern void dm_cvAdd(
  const DmComplexArray * in1,
  const DmComplexArray * in2,
  const DmComplexArray * out,
  int size);
  
extern void dm_cvAdd(
  const DmComplexArraySplit * in1,
  const DmComplexArraySplit * in2,
  const DmComplexArraySplit * out,
  int size);



}}

#endif /* defined(__Project__DmComplex__) */
