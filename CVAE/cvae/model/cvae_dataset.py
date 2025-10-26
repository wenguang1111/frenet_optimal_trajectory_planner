import torch
from torch.utils.data import Dataset
import pandas as pd
from PIL import Image
import numpy as np
import os
import logging
import time
import sys

logging.basicConfig(level=logging.INFO, 
                    format="[%(levelname)s] %(message)s",
                    handlers=[logging.StreamHandler(sys.stdout)])

class CVAEDataset(Dataset):
    def __init__(self, targets_path, conditions_path, image_root, image_transform=None):
        targets_df = pd.read_parquet(targets_path)
        conditions_df = pd.read_parquet(conditions_path)

        assert len(targets_df) == len(conditions_df), \
            "Targets and conditions must have the same length."
            
        self.target_features = ["t", "d", "lon_velocity"]
        self.cond_features = ["x", "y", "theta", "velocity", "acceleration", "yaw_rate"]

        # Precompute NumPy arrays for fast indexing
        self.targets_np = targets_df[self.target_features].to_numpy(dtype="float32")
        self.conds_np = conditions_df[self.cond_features].to_numpy(dtype="float32")

        # Scenario and time step lists for image paths
        self.scenarios = targets_df["scenario"].tolist()
        self.time_steps = targets_df["time_step"].tolist()

        # Precompute full image paths
        self.image_paths = [
            os.path.join(image_root, scenario, f"time_step_{time_step}.png")
            for scenario, time_step in zip(self.scenarios, self.time_steps)
        ]

        self.image_transform = image_transform

    def __len__(self):
        return len(self.targets_np)

    def __getitem__(self, idx):
        ### bottleneck here in image loading ###
        # s_time = time.time()
        target = torch.from_numpy(self.targets_np[idx])
        cond = torch.from_numpy(self.conds_np[idx])

        img = Image.open(self.image_paths[idx]).convert("RGB")
        if self.image_transform:
            img = self.image_transform(img)
        # elapsed = time.time() - s_time
        # logging.info(f"Loaded sample {idx} in {elapsed:.4f} seconds.")
        return target, cond, img
