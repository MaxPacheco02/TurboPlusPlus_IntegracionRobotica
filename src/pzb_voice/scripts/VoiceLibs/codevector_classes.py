import numpy as np
import librosa
from spectrum import poly2lsf, lsf2poly
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple
import json
import pickle
import warnings
import logging
import sys

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

logger = setup_logger(name="classes")


@dataclass
class RawDataMFCC:
    """Class to store individual frame data with MFCC coefficients from audio recordings."""
    raw_samples: np.ndarray
    sample_rate: int = 16000  # 16kHz default
    n_channels: int = 1       # Mono default
    frame_duration_ms: float = 20.0  # 20ms frame default
    mfcc: np.ndarray = field(default_factory=lambda: np.zeros(13))
    parent_centroid_id: int = 0
    generation: int = 0
    frame_number: int = 0
    recording: str = ""
    
    def __post_init__(self):
        """Calculate MFCC coefficients upon initialization."""
        if len(self.raw_samples) > 0:
            self.calculate_mfcc()
            
            # Ensure all arrays are flattened for consistency
            self.raw_samples = self.raw_samples.flatten()
            self.mfcc= self.mfcc.flatten()
    
    def calculate_mfcc(self):
        """Calculate MFCC coefficients using librosa."""
        try:

            # Use frame length for n_fft since its single frames
            frame_length = len(self.raw_samples)

            # Calculate MFCC for single frame
            mfcc_features = librosa.feature.mfcc(
                y=self.raw_samples.astype(float).flatten(),
                sr=self.sample_rate,
                n_mfcc=13,
                n_fft=frame_length,  
                hop_length=None,  # Process entire frame at once
                center=False,  # Don't pad the frame
                n_mels=26  # Standard number of mel filters
            )
            
            # Extract the MFCC coefficients (should be shape (13, 1))
            self.mfcc= mfcc_features.flatten()
            
        except Exception as e:
            # If MFCC calculation fails, keep default zeros
            print(f"Warning: MFCC calculation failed for frame, using defaults: {e}")
            self.mfcc= np.zeros(13)
    
    def to_dict(self):
        """Convert to dictionary for JSON serialization."""
        return {
            'raw_samples': self.raw_samples.tolist(),
            'sample_rate': self.sample_rate,
            'n_channels': self.n_channels,
            'frame_duration_ms': self.frame_duration_ms,
            'mfcc_vector': self.mfcc.tolist(),
            'parent_centroid_id': self.parent_centroid_id,
            'generation': self.generation,
            'frame_number': self.frame_number,
            'recording': self.recording
        }

    @classmethod
    def from_dict(cls, data):
        """Create RawDataMFCC from dictionary."""
        return cls(
            raw_samples=np.array(data['raw_samples']),
            sample_rate=data['sample_rate'],
            n_channels=data['n_channels'],
            frame_duration_ms=data['frame_duration_ms'],
            mfcc=np.array(data['mfcc_vector']),
            parent_centroid_id=data['parent_centroid_id'],
            generation=data['generation'],
            frame_number=data['frame_number'],
            recording=data['recording']
        )


@dataclass
class CentroidDataMFCC:
    mfcc: np.ndarray = field(default_factory=lambda: np.zeros(13))
    id: int = 0

    def __post_init__(self):
        self.mfcc.flatten()

    def to_dict(self):
        """Convert to dictionary for JSON serialization."""
        return {
            'mfcc': self.mfcc.tolist(),
            'id': self.id
        }
    
    @classmethod
    def from_dict(cls, data):
        """Create CentroidData from dictionary."""
        return cls(
            mfcc=np.array(data['mfcc']),
            id=data['id']
        )


class DataStorage:
    """Class to handle saving and loading data."""
    
    @staticmethod
    def save_raw_data(raw_data_list, filepath: str, print_messages=False):
        """Save raw data to JSON file (works with both RawData, RawDataTraining, RawDataMFCC)."""
        data = [frame.to_dict() for frame in raw_data_list]
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        if print_messages:
            print(f"    Saved {len(raw_data_list)} frames to {filepath}")
    
    @staticmethod
    def load_raw_data(filepath: str, data_type: str = 'auto', print_messages = True):
        """Load raw data from JSON file.
        
        Args:
            filepath: Path to JSON file
            data_type: 'train' for RawData, 'hmm'/'test' for RawDataTraining, 'auto' to detect
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        if not data:  # Empty file
            return []
        
        # Auto-detect type based on data structure
        if data_type == 'auto':
            sample_item = data[0]
            if 'lpc_vector' in sample_item and 'lsf_vector' in sample_item:
                data_type = 'train'
            else:
                data_type = 'hmm'  # or 'test', same class
        
        if data_type == 'train':
            raw_data_list = [RawData.from_dict(frame_data) for frame_data in data]
        else:  # 'hmm' or 'test'
            raw_data_list = [RawDataTraining.from_dict(frame_data) for frame_data in data]
        
        if print_messages:
            print(f"  Loaded {len(raw_data_list)} frames from {filepath}")
        return raw_data_list

    @staticmethod
    def load_raw_data_mfcc(filepath: str, data_type: str = 'auto', print_messages = True):
        """Load raw data from JSON file.
        
        Args:
            filepath: Path to JSON file
            data_type: 'train' , 'hmm'/'test' , 'auto' to detect
        """
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        if not data:  # Empty file
            return []
        
        raw_data_list = [RawDataMFCC.from_dict(frame_data) for frame_data in data]
        
        if print_messages:
            print(f"  Loaded {len(raw_data_list)} frames from {filepath}")
        return raw_data_list

    # @staticmethod
    # def save_centroids(centroids: List[CentroidData], filepath: str):
    #     """Save centroids to JSON file."""
    #     data = [centroid.to_dict() for centroid in centroids]
    #     with open(filepath, 'w') as f:
    #         json.dump(data, f, indent=2)
    #     print(f"Saved {len(centroids)} centroids to {filepath}")
    
    # @staticmethod
    # def load_centroids(filepath: str, print_messages=True) -> List[CentroidData]:
    #     """Load centroids from JSON file."""
    #     with open(filepath, 'r') as f:
    #         data = json.load(f)
    #     centroids = [CentroidData.from_dict(centroid_data) for centroid_data in data]
    #     if print_messages:
    #         print(f"Loaded {len(centroids)} centroids from {filepath}")
    #     return centroids
    
    # @staticmethod
    # def save_generations(generations: List[List[CentroidData]], filepath: str):
    #     """Save all generations to a single JSON file."""
    #     data = []
    #     for gen_centroids in generations:
    #         gen_data = [centroid.to_dict() for centroid in gen_centroids]
    #         data.append(gen_data)
    #     with open(filepath, 'w') as f:
    #         json.dump(data, f, indent=2)
    #     print(f"Saved {len(generations)} generations to {filepath}")
    # 
    # @staticmethod
    # def load_generations(filepath: str) -> List[List[CentroidData]]:
    #     """Load generations from JSON file."""
    #     with open(filepath, 'r') as f:
    #         data = json.load(f)
    #     
    #     generations = []
    #     for gen_data in data:
    #         gen_centroids = [CentroidData.from_dict(centroid_data) for centroid_data in gen_data]
    #         generations.append(gen_centroids)
    #     
    #     print(f"Loaded {len(generations)} generations from {filepath}")
    #     return generations

    @staticmethod
    def save_centroids(centroids: List[CentroidDataMFCC], filepath: str):
        """Save centroids to JSON file."""
        data = [centroid.to_dict() for centroid in centroids]
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved {len(centroids)} centroids to {filepath}")
    
    @staticmethod
    def load_centroids(filepath: str, print_messages=True) -> List[CentroidDataMFCC]:
        """Load centroids from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        centroids = [CentroidDataMFCC.from_dict(centroid_data) for centroid_data in data]
        if print_messages:
            print(f"Loaded {len(centroids)} centroids from {filepath}")
        return centroids
    
    @staticmethod
    def save_generations(generations: List[List[CentroidDataMFCC]], filepath: str):
        """Save all generations to a single JSON file."""
        data = []
        for gen_centroids in generations:
            gen_data = [centroid.to_dict() for centroid in gen_centroids]
            data.append(gen_data)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Saved {len(generations)} generations to {filepath}")
    
    @staticmethod
    def load_generations(filepath: str) -> List[List[CentroidDataMFCC]]:
        """Load generations from JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        generations = []
        for gen_data in data:
            gen_centroids = [CentroidDataMFCC.from_dict(centroid_data) for centroid_data in gen_data]
            generations.append(gen_centroids)
        
        print(f"Loaded {len(generations)} generations from {filepath}")
        return generations
    
    @staticmethod
    def save_data_binary(data, filepath: str, print_messages =False):
        """Save data using pickle for faster loading (alternative to JSON)."""
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)
        if print_messages:
            print(f"    Saved data to {filepath} (binary format)")
    
    @staticmethod
    def load_data_binary(filepath: str):
        """Load data from pickle file."""
        with open(filepath, 'rb') as f:
            data = pickle.load(f)
        print(f"Loaded data from {filepath} (binary format)")
        return data


