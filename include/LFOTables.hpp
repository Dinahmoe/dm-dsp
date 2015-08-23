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
 * LFOTables.hpp
 * Lookup tables for LFOs
 *  
 * Created by Alessandro Saccoia on 26/09/10.
 * Copyright 2010 keplero.com All rights reserved.
 */

#ifndef _KEPLERO_COMMON_UTILITY_LFOTABLES
#define _KEPLERO_COMMON_UTILITY_LFOTABLES

#include <cmath>
#ifdef DM_PLATFORM_CYGWIN
#define M_PI 3.14159265358979323846
#endif

namespace dinahmoe {
namespace dsp {


template <typename T>
void SineTable(T* dest_, const unsigned int size_, const float offset_ = .0F, const float amp_ = 1.F)
{
  T actualSize = size_ - 1;
	for (unsigned int i = 0; i < size_; ++i) {
		dest_[i] = offset_ + (amp_ * sin(2* M_PI * (static_cast<T>(i) / actualSize)));
	}
  dest_[size_ - 1] = dest_[0];
}

template <typename T>
void TriangleTable(T* _dest, const unsigned int _size, const float offset_ = .0F, const float amp_ = 1.F)
{	
	T increment = 1.0 / (T)(_size >> 1);
	for (unsigned int i = 0; i < _size; ++i)
	{		
		_dest[i] = offset_ + amp_ * (i > (_size >> 1)) ? (0.0 + increment) : (1 - increment);
	}
}

template <typename T>
void SawtoothTable(T* _dest, const unsigned int _size, const float offset_ = .0F, const float amp_ = 1.F)
{
	T increment = 1.0 / (T)_size;
	T v_step = 0.0;
	for (unsigned int i = 0; i < _size; ++i, v_step += increment)
	{
		_dest[i] = offset_ + amp_ * v_step;		
	}
}

template <typename T>
void SquareTable(T* _dest, const unsigned int _size, const float offset_ = .0F, const float amp_ = 1.F)
{
	for (unsigned int i = 0; i < _size; ++i)
	{
		_dest[i] = offset_ + amp_ * (i > (_size >> 1)) ? 1 : 0;
	}
}

}
}

#endif // _KEPLERO_COMMON_UTILITY_LFOTABLES