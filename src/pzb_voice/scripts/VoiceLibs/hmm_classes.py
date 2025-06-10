from dataclasses import dataclass
import json
import numpy as np
from typing import List
import os


@dataclass
class HMMTrained:
    """Trained HMM model for a specific word."""
    states: int
    symbols: int
    A: np.ndarray  # Transition matrix
    B: np.ndarray  # Emission matrix
    Pi: np.ndarray  # Initial state probabilities
    word: str  # Word name (e.g., "begin", "finish")
    
    def __init__(self, states: int, symbols: int, A: np.ndarray, B: np.ndarray, Pi: np.ndarray, word: str):
        self.states = states
        self.symbols = symbols
        self.A = A
        self.B = B
        self.Pi = Pi
        self.word = word

    def to_dict(self):
        """Convert to dictionary for JSON serialization."""
        return {
            'states': self.states,
            'symbols': self.symbols,
            'A': self.A.tolist(),
            'B': self.B.tolist(),
            'Pi': self.Pi.tolist(),
            'word': self.word
        }

    @classmethod
    def from_dict(cls, data):
        """Create HMMTrained from dictionary."""
        return cls(
            states=data['states'],
            symbols=data['symbols'],
            A=np.array(data['A']),
            B=np.array(data['B']),
            Pi=np.array(data['Pi']),
            word=data['word']
        )

class DataStorageHMM:
    """Storage utilities for HMM models."""
    
    @staticmethod
    def save_hmm(hmm: HMMTrained, base_dir: str = "../Data/ResultsHMM", print_messages=True):
        """Save HMM model to JSON file."""
        os.makedirs(base_dir, exist_ok=True)
        filepath = os.path.join(base_dir, f"{hmm.word}.json")
        
        with open(filepath, 'w') as f:
            json.dump(hmm.to_dict(), f, indent=2)
        if print_messages:
            print(f"Saved HMM for word '{hmm.word}' to {filepath}")
    
    @staticmethod
    def load_hmm(word: str, base_dir: str = "../Data/ResultsHMM", print_messages=True) -> HMMTrained:
        """Load HMM model from JSON file."""
        filepath = os.path.join(base_dir, f"{word}.json")
        
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        hmm = HMMTrained.from_dict(data)
        if print_messages:
            print(f"Loaded HMM for word '{word}' from {filepath}")
        return hmm
    
    @staticmethod
    def load_all_hmms(base_dir: str = "../Data/ResultsHMM",print_messages=True) -> List[HMMTrained]:
        """Load all HMM models from directory."""
        hmms = []
        
        if not os.path.exists(base_dir):
            print(f"Directory {base_dir} does not exist")
            return hmms
        
        for filename in os.listdir(base_dir):
            if filename.endswith('.json'):
                word = filename[:-5]  # Remove .json extension
                try:
                    hmm = DataStorageHMM.load_hmm(word, base_dir, print_messages)
                    hmms.append(hmm)
                except Exception as e:
                    print(f"Error loading HMM for word '{word}': {e}")

        if print_messages:
            print(f"Loaded {len(hmms)} HMM models total")
        return hmms

