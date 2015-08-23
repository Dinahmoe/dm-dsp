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
 
 /**
 @date 8/13/2015
 @author Alessandro Saccoia
 @copyright Alessandro Saccoia 2012-2015
 */

#ifndef _AudioCallbackHelper_hpp_
#define _AudioCallbackHelper_hpp_

#include "RingBuffer.hpp"

//complitely arbitrary, we normally have memory to waste
#define MAX_BUFFER_SIZE 15000

/**
 *  This class provide a buffering mechanism to help with the varying
 *  size of the buffers requested by some hosts
 */
 template <class SAMPLE_T, int MAX_CHANNELS>
class AudioCallbackHelper {
 public:
  typedef void* AudioCallbackArgs;
  typedef size_t (*AudioCallback)
   ( SAMPLE_T** dataIn
   , size_t channelsIn
   , SAMPLE_T** dataOut
   , size_t channelsOut
   , size_t count
   , AudioCallbackArgs args);
  
  AudioCallbackHelper(AudioCallback callback, AudioCallbackArgs callbackArgs)
    : mCallback(callback)
    , mCallbackArgs(callbackArgs)
    , mInputChannels(0)
    , mOutputChannels(0)
    , mHostBufferSize(0)
    , mProcessingBufferSize(0) {
    mInputBuffer = new float*[MAX_CHANNELS];
    mOutputBuffer = new float*[MAX_CHANNELS];
    for (unsigned int i = 0; i < MAX_CHANNELS; ++i) {
      mInputRingBuffers[i].resize(MAX_BUFFER_SIZE);
      mOutputRingBuffers[i].resize(MAX_BUFFER_SIZE);
      mInputBuffer[i] = new float[MAX_BUFFER_SIZE];
      mOutputBuffer[i] = new float[MAX_BUFFER_SIZE];
    }
  }
  
  ~AudioCallbackHelper() {
    for (unsigned int i = 0; i < MAX_CHANNELS; ++i) {
      delete [] mInputBuffer[i];
      delete [] mOutputBuffer[i];
    }
    delete [] mInputBuffer;
    delete [] mOutputBuffer;
  }
  
  /** This method should be called whenever the buffersize or channel config changes */
  void setup(size_t channelsIn, size_t channelsOut, size_t hostBufferSize, size_t processingBufferSize) {
    mInputChannels = channelsIn;
    mOutputChannels = channelsOut;
    mHostBufferSize = hostBufferSize;
    mProcessingBufferSize = processingBufferSize;
    for (unsigned int i = 0; i < mInputChannels; ++i) {
      mInputRingBuffers[i].reset();
      mInputRingBuffers[i].zero(mHostBufferSize);
    }
    for (unsigned int i = 0; i < mOutputChannels; ++i) {
      mOutputRingBuffers[i].reset();
      mOutputRingBuffers[i].zero(mHostBufferSize);
    }
    mCurrentRingBufferSize = mHostBufferSize;
  }
  
  /** This is the latency of the algorithm */
  size_t getLatencyInSamples() { return mHostBufferSize; }
  
  
  /** Call this method to reset the internal buffers when the configuration changes */
  void reset() {
    for (unsigned int i = 0; i < mInputChannels; ++i) {
      mInputRingBuffers[i].reset();
      mInputRingBuffers[i].zero(mHostBufferSize);
    }
    for (unsigned int i = 0; i < mOutputChannels; ++i) {
      mOutputRingBuffers[i].reset();
      mOutputRingBuffers[i].zero(mHostBufferSize);
    }
  }
  
  /** This callback doesn't specify the channels, they should have been set with setup */
  size_t process(SAMPLE_T** dataIn, SAMPLE_T** dataOut, size_t numberOfFrames) {
    mCurrentRingBufferSize += numberOfFrames;
    for (int i = 0; i < mInputChannels; ++i) {
      mInputRingBuffers[i].write(dataIn[i], numberOfFrames);
    }
    while (mCurrentRingBufferSize >= mProcessingBufferSize) {
      for (int i = 0; i < mInputChannels; ++i) {
        mInputRingBuffers[i].read(mInputBuffer[i], mProcessingBufferSize, true);
      }
      mCallback((float**)mInputBuffer, mInputChannels, (float**)mOutputBuffer, mOutputChannels, mProcessingBufferSize, mCallbackArgs);
      for (int i = 0; i < mOutputChannels; ++i) {
        mOutputRingBuffers[i].write(mOutputBuffer[i], mProcessingBufferSize);
      }
      mCurrentRingBufferSize -= mProcessingBufferSize;
    }
    for (int i = 0; i < mOutputChannels; ++i) {
      mOutputRingBuffers[i].read(dataOut[i], numberOfFrames, true);
    }
    return numberOfFrames;
  }


 private:
  AudioCallback mCallback;
  AudioCallbackArgs mCallbackArgs;
  dinahmoe::RingBuffer<SAMPLE_T> mInputRingBuffers[MAX_CHANNELS];
  dinahmoe::RingBuffer<SAMPLE_T> mOutputRingBuffers[MAX_CHANNELS];
  SAMPLE_T **mInputBuffer;
  SAMPLE_T **mOutputBuffer;
  size_t mInputChannels;
  size_t mOutputChannels;
  size_t mHostBufferSize;
  size_t mProcessingBufferSize;
  size_t mCurrentRingBufferSize;
};


#endif
