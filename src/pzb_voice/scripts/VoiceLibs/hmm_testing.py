import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hmmlearn import hmm
from typing import List, Dict, Tuple
import numpy as np
import logging
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, classification_report

from codevector_classes import DataStorage, RawDataMFCC
from hmm_training import get_observations, calculate_log_alpha, safe_log, log_sum_exp
from hmm_classes import HMMTrained


def calculate_log_likelihood(recording_observations: np.ndarray, hmm: HMMTrained) -> float:
    """
    Calculate log-likelihood of a single recording given an HMM model using forward algorithm.
    Uses log-space calculations to prevent underflow.
    
    Args:
        recording_observations: Single recording observation sequence (np.ndarray)
                               Shape: (T,) where T is sequence length
                               Example: [45, 12, 203, 67, ...] representing centroid IDs
        hmm: Trained HMM model containing linear-space matrices
    
    Returns:
        float: log P(O|λ) - log-likelihood for this recording (for direct comparison)
    """
    T = len(recording_observations)  # Number of timesteps in this recording
    N = hmm.states  # Number of states
    
    # Convert HMM matrices from linear to log space
    log_a_matrix = safe_log(hmm.A)  # Transition matrix in log space
    log_b_matrix = safe_log(hmm.B)  # Emission matrix in log space
    log_pi = safe_log(hmm.Pi)       # Initial probabilities in log space
    
    # Initialize log_alpha matrix in log space
    log_alpha = np.full((N, T), float('-inf'))
    
    # Initialize first timestep (t=0) in log space
    obs_symbol = recording_observations[0]
    for state in range(N):
        if log_pi[state] != float('-inf') and log_b_matrix[state, obs_symbol] != float('-inf'):
            log_alpha[state, 0] = log_pi[state] + log_b_matrix[state, obs_symbol]
        else:
            log_alpha[state, 0] = float('-inf')
    
    # Wrap single recording for calculate_log_alpha interface
    observations_list = [recording_observations]
    recording_idx = 0
    
    # Forward pass for remaining timesteps using calculate_log_alpha
    for timestep in range(1, T):
        for state in range(N):
            calculate_log_alpha(timestep, state, log_alpha, recording_idx, 
                              observations_list, log_a_matrix, log_b_matrix, N)
    
    # Calculate log P(O|lambda) for this recording
    # Sum the final alpha values in log space using log_sum_exp
    final_log_alphas = []
    for state in range(N):
        if log_alpha[state, T-1] != float('-inf'):
            final_log_alphas.append(log_alpha[state, T-1])
    
    if final_log_alphas:
        log_probability_O_given_lambda = log_sum_exp(np.array(final_log_alphas))
    else:
        log_probability_O_given_lambda = float('-inf')
    
    return log_probability_O_given_lambda
