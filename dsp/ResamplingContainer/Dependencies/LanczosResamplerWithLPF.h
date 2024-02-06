// File: ResamplingContainer.h
// Created Date: Monday February 5th 2024
// Author: Mikko Honkala (mikko.honkala@gmail.com)

// A container for real-time resampling using a Low pass filtered Lanczos anti-aliasing filter

#pragma once

#include <iostream>
#include <functional>
#include <cmath>

#include "LanczosResampler.h"
#include "AudioDSPTools/dsp/RecursiveLinearFilter.h"


namespace dsp
{
/**
 * Extends the LanczosResampler class by integrating a LowPassBiquad filter to reduce aliasing.
 * This subclass applies a low-pass filter to the input audio signal before performing
 * the Lanczos resampling process. The low-pass filtering stage is crucial for removing
 * high-frequency content that could cause aliasing when the signal is downsampled,
 * ensuring a cleaner output signal with minimal distortion.
 *
 * The LowPassBiquad filter's parameters, such as the cutoff frequency, are set during
 * the object's construction, allowing for flexible adaptation to various audio processing
 * scenarios. This makes the LanczosResamplerWithLPF class particularly useful in applications
 * where audio quality and fidelity are paramount, providing high-quality resampling
 * with integrated anti-aliasing filtering.
 *
 * Template parameters:
 * - T: The data type of the audio samples (e.g., float or double).
 * - NCHANS: The number of audio channels to process (e.g., 1 for mono, 2 for stereo).
 * - A: The filter size parameter of the Lanczos resampler, affecting quality and latency.
 */
template<typename T = double, int NCHANS = 2, size_t A = 12>
class LanczosResamplerWithLPF : public LanczosResampler<T, NCHANS, A> {
public:
    recursive_linear_filter::LowPassBiquad lowPassFilter1, lowPassFilter2; // Two instances for cascaded filtering

    bool applyLPF = false; // Conditionally apply LPF

    // Constructor
    LanczosResamplerWithLPF(float inputRate, float outputRate)
    : LanczosResampler<T, NCHANS, A>(inputRate, outputRate) {
        // Compute cutoff frequency based on the output rate, set to just below Nyquist
        float cutoffFrequency1 = outputRate / 2.0 * 0.9; // 90% of Nyquist frequency
        // Adjusted cutoff frequency for the second filter, increased by 2%
        float cutoffFrequency2 = outputRate / 2.0 * 0.918; // 91.8% of Nyquist frequency (0.9 * 1.02 = 0.918)

        // Only initialize and apply LPF if downsampling
        if (outputRate < inputRate) {
            double qualityFactor = 0.707; // Common choice for a Butterworth filter
            double gainDB = 0.0; // No gain change for a low-pass filter
            recursive_linear_filter::BiquadParams params(inputRate, cutoffFrequency1, qualityFactor, gainDB);
            lowPassFilter1.SetParams(params); // Set parameters for the first filter
            // Set slightly adjusted parameters for the second filter for a steeper slope
            recursive_linear_filter::BiquadParams params2(inputRate, cutoffFrequency2, qualityFactor, gainDB);
            lowPassFilter2.SetParams(params2);
            
            applyLPF = true;
        }
    }

    // Override the PushBlock method to conditionally apply LPF
    void PushBlock(T** inputs, size_t nFrames) override {
        if (applyLPF) {
            // Apply the low-pass filter to the inputs before resampling
            DSP_SAMPLE** dspInputs = reinterpret_cast<DSP_SAMPLE**>(inputs);
            DSP_SAMPLE** filteredOutputs1 = lowPassFilter1.Process(dspInputs, NCHANS, nFrames); // First filtering pass
            DSP_SAMPLE** filteredOutputs2 = lowPassFilter2.Process(filteredOutputs1, NCHANS, nFrames); // Second filtering pass for a steeper slope

            LanczosResampler<T, NCHANS, A>::PushBlock(reinterpret_cast<T**>(filteredOutputs2), nFrames);
        } else {
            // Directly call base class PushBlock without LPF
            LanczosResampler<T, NCHANS, A>::PushBlock(inputs, nFrames);
        }
    }
};

}; // namespace dsp
