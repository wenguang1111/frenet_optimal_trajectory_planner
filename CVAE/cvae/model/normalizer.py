import numpy as np
import pickle
import os
from sklearn.preprocessing import StandardScaler
import logging

class Normalizer:
    """
    Handles normalization for CVAE trajectory data with separate scalers
    for targets and conditions.
    """
    def __init__(self):
        self.target_scaler = StandardScaler()
        self.cond_scaler = StandardScaler()
        self.is_fitted = False
        
    def fit(self, targets_data, conditions_data):
        """
        Fit the scalers on training data.
        
        Args:
            targets_data: numpy array of shape (N, 3) for [t, d, lon_velocity]
            conditions_data: numpy array of shape (N, 6) for [x, y, theta, velocity, acceleration, yaw_rate]
        """
        logging.info("Fitting normalizers on training data...")
        self.target_scaler.fit(targets_data)
        self.cond_scaler.fit(conditions_data)
        self.is_fitted = True
        
        # Log statistics
        logging.info(f"Target features - Mean: {self.target_scaler.mean_}, Std: {self.target_scaler.scale_}")
        logging.info(f"Condition features - Mean: {self.cond_scaler.mean_}, Std: {self.cond_scaler.scale_}")
        
    def transform_targets(self, targets):
        """Transform targets to normalized space"""
        if not self.is_fitted:
            raise RuntimeError("Normalizer must be fitted before transform")
        return self.target_scaler.transform(targets).astype(np.float32)
    
    def transform_conditions(self, conditions):
        """Transform conditions to normalized space"""
        if not self.is_fitted:
            raise RuntimeError("Normalizer must be fitted before transform")
        return self.cond_scaler.transform(conditions).astype(np.float32)
    
    def inverse_transform_targets(self, normalized_targets):
        """
        Reverse normalization for targets (CRITICAL for inference)
        
        Args:
            normalized_targets: tensor or numpy array from CVAE output
        Returns:
            Real-scale targets for trajectory planning
        """
        if not self.is_fitted:
            raise RuntimeError("Normalizer must be fitted before inverse_transform")
        
        # Handle both numpy and torch tensors
        if hasattr(normalized_targets, 'cpu'):
            # PyTorch tensor
            normalized_targets = normalized_targets.cpu().numpy()
        
        return self.target_scaler.inverse_transform(normalized_targets)
    
    def inverse_transform_conditions(self, normalized_conditions):
        """Reverse normalization for conditions"""
        if not self.is_fitted:
            raise RuntimeError("Normalizer must be fitted before inverse_transform")
        
        if hasattr(normalized_conditions, 'cpu'):
            normalized_conditions = normalized_conditions.cpu().numpy()
        
        return self.cond_scaler.inverse_transform(normalized_conditions)
    
    def save(self, save_dir):
        """
        Save fitted scalers to disk.
        
        Args:
            save_dir: directory to save normalizer files
        """
        if not self.is_fitted:
            raise RuntimeError("Cannot save unfitted normalizer")
        
        os.makedirs(save_dir, exist_ok=True)
        
        # Save scalers
        target_scaler_path = os.path.join(save_dir, 'target_scaler.pkl')
        cond_scaler_path = os.path.join(save_dir, 'cond_scaler.pkl')
        
        with open(target_scaler_path, 'wb') as f:
            pickle.dump(self.target_scaler, f)
        
        with open(cond_scaler_path, 'wb') as f:
            pickle.dump(self.cond_scaler, f)
        
        # Also save as readable text for verification
        stats_path = os.path.join(save_dir, 'normalization_stats.txt')
        with open(stats_path, 'w') as f:
            f.write("TARGET SCALER STATISTICS\n")
            f.write("=" * 50 + "\n")
            f.write(f"Mean: {self.target_scaler.mean_}\n")
            f.write(f"Std: {self.target_scaler.scale_}\n\n")
            
            f.write("CONDITION SCALER STATISTICS\n")
            f.write("=" * 50 + "\n")
            f.write(f"Mean: {self.cond_scaler.mean_}\n")
            f.write(f"Std: {self.cond_scaler.scale_}\n")
        
        logging.info(f"Normalizer saved to {save_dir}")
    
    @classmethod
    def load(cls, save_dir):
        """
        Load fitted scalers from disk.
        
        Args:
            save_dir: directory containing saved normalizer files
        Returns:
            CVAENormalizer instance with loaded scalers
        """
        normalizer = cls()
        
        target_scaler_path = os.path.join(save_dir, 'target_scaler.pkl')
        cond_scaler_path = os.path.join(save_dir, 'cond_scaler.pkl')
        
        if not os.path.exists(target_scaler_path) or not os.path.exists(cond_scaler_path):
            raise FileNotFoundError(f"Scaler files not found in {save_dir}")
        
        with open(target_scaler_path, 'rb') as f:
            normalizer.target_scaler = pickle.load(f)
        
        with open(cond_scaler_path, 'rb') as f:
            normalizer.cond_scaler = pickle.load(f)
        
        normalizer.is_fitted = True
        
        logging.info(f"Normalizer loaded from {save_dir}")
        logging.info(f"Target mean: {normalizer.target_scaler.mean_}")
        logging.info(f"Target std: {normalizer.target_scaler.scale_}")
        logging.info(f"Conditions mean: {normalizer.cond_scaler.mean_}")
        logging.info(f"Conditions std: {normalizer.cond_scaler.scale_}")
        
        return normalizer