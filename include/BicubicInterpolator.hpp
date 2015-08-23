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
 *  BicubicInterpolator.h
 *
 *  Bicubic interpolation between 0 and the given abscissa.
 *  f(0) is considered to be equal to 0
 *
 *  Created by Alessandro Saccoia on 5/26/12.
 *
 */


#ifndef __BICUBICINTERPOLATOR_H__
#define __BICUBICINTERPOLATOR_H__

template <class FTYPE>
class BicubicInterpolator {
public:
  // if used to compute the gain in the knee region:
  // xmax_ = knee in db
  // ymax_ = (knee/2) + ((knee/2)/ratio)
  // y1max_ = 1/ratio
  inline void setCoefficients(FTYPE xmax_, FTYPE ymax_, FTYPE y1max_);  
  inline FTYPE interpolate(FTYPE in_);

private:
  FTYPE m_xmax;
  FTYPE m_a;
  FTYPE m_b;
  FTYPE m_c;
  FTYPE m_d;
	
};


template <class FTYPE>
inline void BicubicInterpolator<FTYPE>::setCoefficients(FTYPE xmax_, FTYPE ymax_, FTYPE y1max_) {
  m_xmax = xmax_;
  // scale down everything to be in the range 0-1, scale up at the end
  const FTYPE f0 = 0;
  const FTYPE f1 = ymax_ / xmax_;
  const FTYPE f1_0 = 1;
  const FTYPE f1_1 = y1max_;
  m_a = -2 * f1 + f1_0 + f1_1;
  m_b = 3 * f1 - 2 * f1_0 - f1_1;
  m_c = f1_0;
  m_d = f0;
}

template <class FTYPE>
inline FTYPE BicubicInterpolator<FTYPE>::interpolate(FTYPE in_) {
  //scale down the input in the range 0-1
  //y = a * x * x * x + b * x * x + c * x;
  FTYPE inScaled = in_ / m_xmax;
  FTYPE ret = m_c * inScaled;
  inScaled *= inScaled;
  ret += m_b * inScaled;
  inScaled *= inScaled;
  ret += m_a * inScaled;
  ret *= m_xmax;
  return ret;
}


#endif // __BICUBICINTERPOLATOR_H__

