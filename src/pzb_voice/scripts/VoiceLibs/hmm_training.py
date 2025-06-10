import json
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix
from typing import List, Dict, Tuple
import sys
import logging
import math

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmm_classes import HMMTrained, DataStorageHMM
from codevector_classes import RawDataMFCC, CentroidDataMFCC 


def safe_log(x):
    """Safely compute log, returning -inf for values <= 0"""
    if isinstance(x, np.ndarray):
        result = np.full_like(x, float('-inf'), dtype=float)
        mask = x > 0
        result[mask] = np.log(x[mask])
        return result
    else:
        return math.log(x) if x > 0 else float('-inf')

def log_sum_exp(log_probs):
    """Numerically stable log(sum(exp(log_probs))) for arrays or scalars"""
    if isinstance(log_probs, np.ndarray):
        # Remove -inf values for computation
        finite_mask = log_probs != float('-inf')
        if not np.any(finite_mask):
            return float('-inf')
        
        finite_probs = log_probs[finite_mask]
        max_val = np.max(finite_probs)
        return max_val + math.log(np.sum(np.exp(finite_probs - max_val)))
    else:
        # Handle scalar case
        return log_probs if log_probs != float('-inf') else float('-inf')


def get_observations(recordings: List[List[RawDataMFCC]], centroids: List[CentroidDataMFCC]) -> list[np.ndarray]:
    """
    Convert RawDataMFCC recordings to observation sequences using vector quantization.
    
    Args:
        recordings: List of List[RawDataMFCC objects] for one word
        centroids: List of CentroidDataMFCC objects (code vector)
    
    Returns:
        list[np.ndarray]: Array of centroid indices for each frame in each recording
    """
    observations = []

    for recording in recordings:
        recording_observations = []
        
        for frame in recording:
            # Extract MFCC features (excluding power - index 0)
            frame_mfcc = frame.mfcc[1:]  # Skip index 0 (power)
            
            min_distance = float('inf')
            closest_centroid_id = 0
            
            # Find closest centroid using Euclidean distance
            for centroid_idx, centroid in enumerate(centroids):
                centroid_mfcc = centroid.mfcc[1:]  # Skip index 0 (power)
                
                # Calculate Euclidean distance
                distance = np.linalg.norm(frame_mfcc - centroid_mfcc)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_centroid_id = centroid_idx
            
            recording_observations.append(closest_centroid_id)
        
        observations.append(np.array(recording_observations))
    
    return observations



def calculate_log_alpha(timestep: int, origin_state: int, log_alpha: np.ndarray, recording_idx: int, 
                       observations: List[np.ndarray], log_a_matrix: np.ndarray, log_b_matrix: np.ndarray, N: int):
    """
    Calculate forward variable log_alpha at given timestep and state.
    Pure log-space implementation for numerical stability.
    
    Args:
        timestep: Current time step
        origin_state: State we're calculating alpha for
        log_alpha: Log-space alpha matrix
        recording_idx: Index of current recording
        observations: List of observation sequences
        log_a_matrix: Log-space transition matrix
        log_b_matrix: Log-space emission matrix
        N: Number of states
    """
    # Sum over all previous states in log space
    log_sum_terms = []
    
    for prev_state in range(N):
        # Check if transition is possible (not -inf)
        if (log_alpha[prev_state, timestep-1] != float('-inf') and 
            log_a_matrix[prev_state, origin_state] != float('-inf')):
            
            log_term = log_alpha[prev_state, timestep-1] + log_a_matrix[prev_state, origin_state]
            log_sum_terms.append(log_term)
    
    if log_sum_terms:
        # Use log_sum_exp for numerical stability
        log_sum = log_sum_exp(np.array(log_sum_terms))
        
        # Add emission probability in log space
        obs_symbol = observations[recording_idx][timestep]
        if log_b_matrix[origin_state, obs_symbol] != float('-inf'):
            log_alpha[origin_state, timestep] = log_sum + log_b_matrix[origin_state, obs_symbol]
        else:
            log_alpha[origin_state, timestep] = float('-inf')
    else:
        log_alpha[origin_state, timestep] = float('-inf')
