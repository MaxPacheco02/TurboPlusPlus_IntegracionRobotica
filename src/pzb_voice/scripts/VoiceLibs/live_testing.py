import sounddevice as sd
from scipy.io.wavfile import write, read
import wavio as wv
import logging
import sys
import numpy as np
import os 
from typing import List
import matplotlib.pyplot as plt
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from preemphasis import filter_signal, hamming_window, slice_signal
from codevector_classes import RawDataMFCC, DataStorage 
from hmm_classes import DataStorageHMM, HMMTrained
from hmm_training import get_observations
from hmm_testing import calculate_log_likelihood

vocabulary = { 
    "backward" : 0,
    "begin" : 1,
    "down" : 2,
    "finish" : 3,
    "forward" : 4,
    "left" : 5,
    "right" : 6,
    "rotate" : 7,
    "stop" : 8,
    "up" : 9
} 

def save_prediction_json(predicted_word, filename='command.json'):
    # Create the result dictionary
    if predicted_word in vocabulary:
        result = {predicted_word: vocabulary[predicted_word]}
    else:
        result = {"unknown": -1}
    
    # Atomic write pattern
    temp_filename = filename + '.tmp'
    
    try:
        # Write to temporary file first
        with open(temp_filename, 'w') as f:
            json.dump(result, f)
        
        # Atomic rename
        os.rename(temp_filename, filename)
        logger.info(f"Saved prediction: {result}")
        
    except Exception as e:
        logger.error(f"Error saving prediction: {e}")
        # Clean up temp file if it exists
        if os.path.exists(temp_filename):
            os.remove(temp_filename)

def setup_logger(name=__name__, level=logging.DEBUG):
    # Create logger
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Avoid adding multiple handlers if logger already exists
    if logger.handlers:
        return logger
    
    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    
    # Create formatter with file, method, and line information
    formatter = logging.Formatter(
        fmt=' %(levelname)-8s | %(filename)s:%(lineno)d | %(funcName)s() | %(message)s',
    )
    
    # Add formatter to handler
    console_handler.setFormatter(formatter)
    
    # Add handler to logger
    logger.addHandler(console_handler)
    
    return logger

logger = setup_logger(name="live_testing")



def display_graphs(y_space1: np.ndarray, y_space2: np.ndarray, y_space3: np.ndarray, y_space4: np.ndarray, idx: List[int]):
    y_space1 = y_space1.flatten()
    y_space2 = y_space2.flatten()
    y_space3 = y_space3.flatten()
    y_space4 = y_space4.flatten()

    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    x1 = np.arange(1,y_space1.shape[0]+1)
    x2 = np.arange(1,y_space2.shape[0]+1)
    x3 = np.arange(1,y_space3.shape[0]+1)
    x4 = np.arange(1,y_space4.shape[0]+1)


    ax1.plot(x1, y_space1)
    ax1.set_title('Original')
    ax1.set_ylabel('Voltage')

    ax2.plot(x2, y_space2)
    ax2.set_title('Filtered')
    ax2.set_ylabel('Voltage')

    logger.info(f"x3: {x3.shape}, idx: {idx}")
    ax3.plot(x2, y_space2)
    ax3.axvline(x=x2[idx[0]], color='r', linestyle='--')
    ax3.axvline(x=x2[idx[1]], color='r', linestyle='--')
    ax3.set_title('Trimmed')
    ax3.set_ylabel('Voltage')


    ax4.plot(x4, y_space4)
    ax4.set_title('Hamming')
    ax4.set_ylabel('Voltage')

    plt.tight_layout()
    plt.subplots_adjust(hspace=0.3)

    plt.show()


def do_preemphasis(signal_raw: np.ndarray, framerate=16000):
    nframes = signal_raw.shape[0]
    print(f"frames: {nframes}")
    output = filter_signal(signal_raw, nframes)
    signal_filtered = output["filtered_signal"]
    logger.info(f"filtered: {signal_filtered.shape}")

    # errasing_start_samples = signal_filtered[100:]

    output = slice_signal(signal_filtered, framerate, nframes)
    trimmed_signal = output["trimmed_signal"]
    start_idx = output["start_idx"]
    finish_idx = output["finish_idx"]
    logger.info(f"trimmed: {trimmed_signal.shape}")

    output = hamming_window(trimmed_signal, trimmed_signal.shape[0])
    # output = hamming_window(signal_filtered, signal_filtered.shape[0])
    signal_hamming = output["hamming_signal"]
    logger.info(f"hamming: {signal_hamming.shape}")


    display_graphs(signal_raw, signal_filtered, trimmed_signal, signal_hamming, [start_idx, finish_idx])
    return signal_hamming


def record_auido(filename = "recording0", save_file = False, amplify = True):
    freq = 16000
    duration = 1.5

    recording = sd.rec(int(duration * freq), samplerate=freq, channels=1)

    print(f"Recording Audio...")
    sd.wait()

    # Add this after sd.wait()
    if amplify:
        rms = np.sqrt(np.mean(recording**2))
        target_rms = 0.3  # Adjust as needed
        if rms > 0:
            recording = recording * (target_rms / rms)
            recording = np.clip(recording, -1.0, 1.0)


    if save_file:
        if os.path.exists(f"{filename}.wav"):
            os.remove(f"{filename}.wav")
            print(f"Deleting {filename}")

        write(f"{filename}.wav", freq, recording)
        print(f"Saving file {filename}")

    return recording


def split_into_frames_with_overlap(audio_data: np.ndarray, frame_size=320, hop_size=160) -> List[np.ndarray]:
    """Split audio data into overlapping frames."""
    frames = []
    
    # Use hop_size for overlap
    for i in range(0, len(audio_data) - frame_size + 1, hop_size):
        frame = audio_data[i:i + frame_size]
        frames.append(frame)
    
    # Handle the last frame if there are remaining samples
    # No padding - just use whatever samples remain
    last_start = len(frames) * hop_size
    if last_start < len(audio_data):
        last_frame = audio_data[last_start:]
        # Only add if it has enough samples to be meaningful (e.g., > 12 for LPC order 12)
        if len(last_frame) > 12:
            frames.append(last_frame)
    
    return frames

def create_mfcc(frames: List[np.ndarray]):
    raw_data_list = []
    for i, frame in enumerate(frames):
        raw_data = RawDataMFCC(
            raw_samples=frame,
            frame_number=i,
            recording="live_testing_recording"
        )

        raw_data_list.append(raw_data)

    return raw_data_list
   
def load_centroids_mfcc(base_dir = "../Data/"):
    storage = DataStorage()
    codevector_dir = os.path.join(base_dir, "CodeVector")
    if os.path.exists(os.path.join(codevector_dir, "codevector.json")):
        print("\nLoading codevector:")
        centroids = storage.load_centroids(os.path.join(codevector_dir, "codevector.json"), print_messages=False)

    return centroids

def load_hmm_models(base_dir="../Data/Eighty-five-percent_20/"):
    all_hmm = DataStorageHMM.load_all_hmms(base_dir=base_dir,print_messages=False)
        
    if not all_hmm:
        print("No trained HMM models found. Please train models first.")
        return
    
    print(f"Loaded {len(all_hmm)} HMM models for words: {[hmm.word for hmm in all_hmm]}")

    return all_hmm


def do_inference(all_hmm: List[HMMTrained], observations: np.ndarray):
    try:
        likelihoods = {}
        for hmm in all_hmm:
            likelihood = calculate_log_likelihood(observations, hmm)
            likelihoods[hmm.word] = likelihood
            
            # if likelihood > best_likelihood:
            #     best_likelihood = likelihood
            #     predicted_word = hmm.word
    except Exception as e:
        logger.info(f"Error during inference {e}")

    return likelihoods 

def print_dict(title: str, my_dict: dict[np.ndarray]):
    print(f"{title}")
    for key in my_dict:
        print(f"{key}: {float(my_dict[key]):.3f}", end=" ")

def main():
    # Record Audio
    recording = record_auido(save_file=False)
    # _, recording = read("recording1.wav")
    # recording = recording.reshape(-1,1)
    # print(f"recording shape: {recording.shape}")

    # Pre processing
    processed_signal = do_preemphasis(recording[700:])

    # Split into frames and get MFCC
    frames = split_into_frames_with_overlap(processed_signal)
    mfcc_frames = create_mfcc(frames)

    # Loading centroids from disk
    centroids = load_centroids_mfcc()

    # Getting observations
    observations = get_observations([mfcc_frames], centroids)

    # Loading HMM models
    all_hmm = load_hmm_models(base_dir="../Data/ResultsLibraryHMM")

    # Obtain P(O|lambda) for all hmms
    likelihoods = do_inference(all_hmm, observations[0])
    sorted_dict = dict(sorted(likelihoods.items(), key=lambda item: item[1], reverse=True))

    print_dict("Likelihoods ", likelihoods)
    print(f"\n")
    print_dict("Sorted: ", sorted_dict)
    print(f"\n")

    # Extract the best prediction
    if sorted_dict:
        predicted_word = list(sorted_dict.keys())[0]
    else:
        predicted_word = "unknown"
    
    return predicted_word

if __name__ == "__main__":
    exit_ = True
    last_prediction = None
    
    while exit_:
        try:
            last_prediction = main()  # Store the returned prediction
        except Exception as e:
            logger.info(f"Error while recording {e}")

        input_exit = input("Enter q to exit...")

        if input_exit == 'q':
            if last_prediction:
                save_prediction_json(last_prediction)
            exit_ = False
