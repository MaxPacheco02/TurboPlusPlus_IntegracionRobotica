import wave
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import os
import librosa
import soundfile as sf
from typing import Tuple
from pathlib import Path

# INFO: Applying `1-(0.95z^-1)` filter
# Output: array_sample array
def filter_signal(signal: np.ndarray, nframes: int):
    top = 10
    filtered_signal = np.zeros((nframes, 1))
    for i in range(nframes-1):
        # print(f"{filtered_signal[i+1, 0]} = {array_sample[i+1, 0]} - {0.95*array_sample[i,0]}")
        filtered_signal[i+1, 0] = signal[i+1, 0] - 0.95*signal[i, 0]
        # if i > top: break

    output = {"filtered_signal" : filtered_signal}
    return output

# INFO: Calculate Start and Finish with Energy and ZCR
# and get trimmed signal. It removes empty space.
# We must divide into 20ms frames with 10ms overlap
# framerate => 1s
# x         => 20ms
# x -> samples per frame
def slice_signal(signal: np.ndarray, framerate: int, nframes: int):
    samples_per_20ms = int(0.02 * framerate)
    samples_per_10ms = int(0.01 * framerate)
    num_frames_in_signal = int((nframes-samples_per_20ms)/samples_per_10ms) + 1

    zcr = np.zeros((num_frames_in_signal,1))
    power = np.zeros((num_frames_in_signal,1))

    # print(f"Num frames in signal: {num_frames_in_signal}")
    for i in range(num_frames_in_signal):
        start = i*samples_per_10ms
        finish = start + samples_per_20ms

        if i == num_frames_in_signal-1:  
            frame_to_evalute = signal[start:-1].reshape(-1,1)
        else:
            frame_to_evalute = signal[start:finish,0].reshape(-1,1)

        # Calculating ZCR = 1/2 * Sum(sign(n) - sing(n-1))
        frame_to_evalute_shifted = np.roll(frame_to_evalute, -1)
        frame_to_evalute_shifted[-1,0] = 0

        differences = np.subtract(np.sign(frame_to_evalute_shifted), np.sign(frame_to_evalute))
        differences[-1,0] = 0
        abs_array = np.abs(differences)
        sum = np.sum(abs_array)
        zcr[i,0] = sum / 2;

        # Calculating Power = Sum(x^2) / x_len
        power[i,0] = np.sum(frame_to_evalute**2)/len(frame_to_evalute)

    # First slice
    max_zcr = 0.03*np.max(zcr) # Thresholds
    max_power = 0.01*np.max(power)

    zcr_upper_array = np.where(zcr>max_zcr, 1, 0)
    power_upper_array = np.where(power>max_power, 1, 0)

    # Second slice
    max_zcr_2 = 0.0001*np.max(zcr) # Thresholds
    max_power_2 = 0.0001*np.max(power)

    zcr_upper_array_2 = np.where(zcr>max_zcr_2, 1, 0)
    power_upper_array_2 = np.where(power>max_power_2, 1, 0)

    # If the frame has zcr and power greater than the Thresholds
    # then the frame will stay
    zcr_and_power_upper = np.logical_and(zcr_upper_array, power_upper_array).astype(int)
    zcr_and_power_upper_2 = np.logical_and(zcr_upper_array_2, power_upper_array_2).astype(int)

    if np.any(zcr_and_power_upper==1):
    # if np.any(power_upper_array==1):
        first = np.where(zcr_and_power_upper[:,0]==1)[0][0]
        last = np.where(zcr_upper_array_2[:,0]==1)[0][-1]
        # first = np.where(power_upper_array[:,0]==1)[0][0]
        # last = np.where(power_upper_array[:,0]==1)[0][-1]

    else:
        print("No 1 founded in ZCR and Power")
        first = 0
        last = len(power_upper_array)

    start_index_trimmed_sampled = first*samples_per_10ms
    finish_index_trimmed_sampled = last*samples_per_10ms

    trimmed_signal = signal[start_index_trimmed_sampled:finish_index_trimmed_sampled]
    
    output = {
        "trimmed_signal" : trimmed_signal,
        "start_idx" : start_index_trimmed_sampled,
        "finish_idx" : finish_index_trimmed_sampled
      }
    return output


# INFO: Applying Hamming window
# NOTE: In this case hamming window and frame are the same length.
# Output: hamming_signal array
def hamming_window(signal: np.ndarray, nframes: int):
    samples_per_hamming = 320
    samples_per_hop = 128
    num_frames_in_signal_hamming = int((nframes-samples_per_hamming)/samples_per_hop) + 1

    indices_array = np.arange(samples_per_hamming)
    hamming_window = 0.54 - 0.46 * np.cos((2 * np.pi * indices_array)/ (samples_per_hamming-1))

    top = nframes
    bottom = 10
    hamming_signal = np.copy(signal)
    # print(f"Num frames in signal hamming: {num_frames_in_signal_hamming}")
    for i in range(num_frames_in_signal_hamming+1):
        start = i*samples_per_hop
        finish = start + samples_per_hamming

        if i == num_frames_in_signal_hamming:  
            last_samples = len(hamming_signal[start:-1,0])
            hamming_signal[start:-1,0] = hamming_signal[start:-1,0] * hamming_window[0:last_samples]
        else:
            hamming_signal[start:finish,0] = hamming_signal[start:finish,0] * hamming_window 

    output = {"hamming_signal" : hamming_signal}
    return output

