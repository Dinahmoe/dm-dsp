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

//  Created by Alessandro Saccoia on 4/9/14.

#ifndef waveshaperTest_WaveshaperAlgorithms_hpp
#define waveshaperTest_WaveshaperAlgorithms_hpp

#include <cmath>
#include <iostream>

namespace dinahmoe {
namespace dsp {

inline float sign(float x_) {
  return x_ == 0.F ? 1.F : fabs(x_) / x_;
}

inline void curveAlgo0(float amount_, unsigned int nSamples_, float* wsTable_) {
  if ((amount_ >= 0) && (amount_ < 1)) {
    float k = 2.F * (float)amount_ / (1 - (float)amount_);
    for (unsigned int i = 0; i < nSamples_; i+=1) {
      float x = (float)i * 2.F / (float)nSamples_ - 1.F;
      wsTable_[i] = (1.F + k) * x / (1.F+ k * fabs(x));
    }
  }
}

inline void curveAlgo1 (float amount_, unsigned int nSamples_, float* wsTable_) {
  for (unsigned int i = 0; i < nSamples_; i+=1) {
    float x = i * 2.F / (float)nSamples_ - 1.F;
    float y = (0.5F * pow((x + 1.4F), 2.F)) - 1.F;
    if (y >= 0.F) {
      y *= 5.8F;
    } else {
      y *= 1.2F;
    }
    wsTable_[i] = tanh(y);
  }
}

inline void curveAlgo2 (float amount_, unsigned int nSamples_, float* wsTable_) {
  amount_ = 1 - amount_;
  for (unsigned int i = 0; i < nSamples_; i+=1) {
    float x = i * 2.F / (float)nSamples_ - 1.F;
    float y;
    if (x < 0) {
      y = -pow(abs(x), amount_ + 0.04);
    } else {
      y = pow(x, amount_);
    }
    wsTable_[i] = tanh(y * 2);
  }
}

inline void curveAlgo3 (float amount_, unsigned int nSamples_, float* wsTable_) {
  amount_ = 1.F - amount_;
  if (amount_ > 0.99F) { amount_ = 0.99F; } // NaN
  for (unsigned int i = 0; i < nSamples_; i+=1) {
    float x = (float)i * 2.F / (float)nSamples_ - 1.F;
    float abx = fabs(x);
    float y = .0F;
    if (abx < amount_) {
      y = abx;
    } else if ( abx > amount_) {
      y = amount_ + (abx-amount_)/(1.F + powf((abx-amount_)/(1.F-amount_),2.F));
    } else if (abx > 1.F) {
      y = abx;
    }
    wsTable_[i] = (float)dinahmoe::dsp::sign(x) * y * (1.F/((amount_+1.F)/2.F)); //normalize
  }
}
//Fixed curve, amount doesn't do anything, the distortion is just from the drive
//function curveAlgo4 (amount, n_samples, ws_table) {
//// fixed curve, amount doesn't do anything, the distortion is just from the drive
//for (float i = 0; i < n_samples; i+=1) {
//var x = i * 2 / n_samples -1;
//if (x < -0.08905) {
//ws_table[i] = (-3/4) * (1-(Math.pow((1-(Math.abs(x) - 0.032857)),12)) + (1/3) * (Math.abs(x) - 0.032847)) + 0.01;
//} else if ( x >= -0.08905 && x < 0.320018) {
//ws_table[i] = (-6.153 * (x * x)) + 3.9375 * x;
//} else {
//ws_table[i] = 0.630035;
//}
//}
//},

//Bitcrushing Algorithm.
inline void curveAlgo5 (float amount_, unsigned int nSamples_, float* wsTable_) {
  // we go from 2 to 16 bits, keep in mind for the UI
  amount_ = 2 + round(amount_ * 14.F);
  // real number of quantization steps divided by 2
  float bits = round(pow(2.0F, amount_ - 1.F));
  for (unsigned int i = 0; i < nSamples_; i+=1) {
    float x = (float)i * 2.F / nSamples_ - 1.F;
    wsTable_[i] = round(x * bits) / bits;
  }
}
}}

#endif
