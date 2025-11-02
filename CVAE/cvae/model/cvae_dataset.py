import torch
from torch.utils.data import Dataset
import pandas as pd
from PIL import Image
import numpy as np
import os
import logging
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from tqdm import tqdm
from normalizer import Normalizer

logging.basicConfig(level=logging.INFO, 
                    format="[%(levelname)s] %(message)s",
                    handlers=[logging.StreamHandler(sys.stdout)])

class CVAEDataset(Dataset):
    def __init__(self, targets_path,
                 conditions_path,
                 image_root,
                 mode,
                 image_transform=None,
                 num_workers=8):
        
        self.mode = mode
        self.normalizer_dir = 'cvae/model/weights/'
        
        targets_df = pd.read_parquet(targets_path)
        conditions_df = pd.read_parquet(conditions_path)
        
        assert len(targets_df) == len(conditions_df), \
            "Targets and conditions must have the same length."
            
        self.target_features = ["t", "d", "lon_velocity"]
        self.cond_features = ["x", "y", "theta", "velocity", "acceleration", "yaw_rate"]

        self.targets_np = targets_df[self.target_features].to_numpy(dtype="float32")
        self.conds_np = conditions_df[self.cond_features].to_numpy(dtype="float32")
        
        self._setup_normalizer()

        self.scenarios = targets_df["scenario"].tolist()
        self.time_steps = targets_df["time_step"].tolist()
        
        self.image_root = image_root
        self.image_transform = image_transform
        
        # Preload all unique images in parallel
        unique_images = list(set(zip(self.scenarios, self.time_steps)))
        logging.info(f"Preloading {len(unique_images)} unique images using {num_workers} workers...")
        
        self.image_cache = {}
        self._preload_images_parallel(unique_images, num_workers)
        
        logging.info(f"Preloading complete. Cache contains {len(self.image_cache)} images.")

    def _load_single_image(self, scenario, time_step):
        """Load and transform a single image"""
        try:
            img_path = os.path.join(self.image_root, scenario, f"time_step_{time_step}.png")
            img = Image.open(img_path).convert("RGB")
            if self.image_transform:
                img = self.image_transform(img)
            return (scenario, time_step), img
        except Exception as e:
            logging.error(f"Error loading {scenario}/time_step_{time_step}.png: {e}")
            return (scenario, time_step), None

    def _preload_images_parallel(self, unique_images, num_workers):
        """Preload images using ThreadPoolExecutor"""
        with ThreadPoolExecutor(max_workers=num_workers) as executor:
            # Submit all tasks
            futures = {
                executor.submit(self._load_single_image, scenario, time_step): (scenario, time_step)
                for scenario, time_step in unique_images
            }
            
            # Process completed tasks with progress bar
            for future in tqdm(as_completed(futures), total=len(futures), desc="Loading images"):
                key, img = future.result()
                if img is not None:
                    self.image_cache[key] = img

    def _setup_normalizer(self):
        """
        Setup normalizer based on mode:
        - train: Fit on this data, save to disk, then transform
        - val/test: Load from disk, then transform
        """
        if self.mode == 'train':
            # ✅ TRAINING MODE: Fit and save normalizer
            logging.info(f"[TRAIN] Fitting normalizer on training data...")
            
            self.normalizer = Normalizer()
            self.normalizer.fit(self.targets_np, self.conds_np)
            
            # Save for later use
            os.makedirs(self.normalizer_dir, exist_ok=True)
            self.normalizer.save(self.normalizer_dir)
            logging.info(f"[TRAIN] Normalizer fitted and saved to {self.normalizer_dir}")
            
            # Transform training data
            self.targets_np = self.normalizer.transform_targets(self.targets_np)
            self.conds_np = self.normalizer.transform_conditions(self.conds_np)
            logging.info(f"[TRAIN] Training data normalized")
            
        elif self.mode in ['val', 'test']:
            # ✅ VALIDATION/TEST MODE: Load normalizer and transform only
            logging.info(f"[{self.mode.upper()}] Loading normalizer from {self.normalizer_dir}...")
            
            self.normalizer = Normalizer.load(self.normalizer_dir)
            
            if not self.normalizer.is_fitted:
                raise RuntimeError(
                    f"Loaded normalizer is not fitted. "
                    f"Make sure you've trained the model first and saved the normalizer."
                )
            
            logging.info(f"[{self.mode.upper()}] Normalizer loaded successfully")
            
            # Transform validation/test data using training statistics
            self.targets_np = self.normalizer.transform_targets(self.targets_np)
            self.conds_np = self.normalizer.transform_conditions(self.conds_np)
            logging.info(f"[{self.mode.upper()}] Data normalized using training statistics")
            
        else:
            raise ValueError(f"Invalid mode: {self.mode}. Must be 'train', 'val', or 'test'")
        
    def __len__(self):
        return len(self.targets_np)

    def __getitem__(self, idx):
        target = torch.from_numpy(self.targets_np[idx])
        cond = torch.from_numpy(self.conds_np[idx])
        
        cache_key = (self.scenarios[idx], self.time_steps[idx])
        img = self.image_cache[cache_key]
        
        return target, cond, img