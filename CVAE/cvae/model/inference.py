import torch
import numpy as np
from cvae.model.model import CVAE, cvae_loss_function
from tqdm import tqdm
import pandas as pd
from torch.utils.tensorboard import SummaryWriter
from cvae.utils.beta_annealer import BetaAnnealer
from cvae.model.cvae_dataset import CVAEDataset
from cvae.model.mask_background import MaskBackground
from torchvision import transforms
from torch.optim.lr_scheduler import LambdaLR
import sys
import logging

logging.basicConfig(level=logging.INFO, 
                    format="[%(levelname)s] %(message)s",
                    handlers=[logging.StreamHandler(sys.stdout)])

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
logging.info(f"Using device: {device}")

batch_size = 1024
img_features = 64
img_dim = 128
z_dim = 32
x_dim = 3
c_dim = 6 + img_features  # states + img features
h_Q_dim = 64
h_P_dim = 64

num_workers = 16  # for data loading

log_every = 10

# import data
test_imgs_root = 'cvae/data/data_v2/test/imgs/'
targets_test_path = 'cvae/data/data_v2/test/x_test.parquet'
conditions_test_path = 'cvae/data/data_v2/test/c_test.parquet'

imgs_transforms = transforms.Compose([
    transforms.Resize((img_dim, img_dim)),
    transforms.ToTensor(),  # Converts to [0, 1] (normalized)
    MaskBackground()
])

# prepare datasets and dataloaders
test_dataset = CVAEDataset(
    targets_path=targets_test_path,
    conditions_path=conditions_test_path,
    image_root=test_imgs_root,
    mode='test',
    image_transform=imgs_transforms,
    num_workers=num_workers
)

test_dataloader = torch.utils.data.DataLoader(test_dataset, 
                                             batch_size=batch_size,
                                             pin_memory=True,
                                             num_workers=num_workers,
                                             shuffle=True)

# Load the model
model = CVAE(x_dim, c_dim, z_dim)
model.load_state_dict(torch.load('cvae/model/weights/cvae_model_lr_5e-06_batch_1024_epochs_20_zdim_32.pth'))
model = model.to(device)
model.eval()

# s_time = time.time()
with torch.inference_mode():
    cum_loss = 0.0
    for batch in test_dataloader:
        x, c, img = batch
        x, c, img = x.to(device, non_blocking=True), c.to(device, non_blocking=True), img.to(device, non_blocking=True)
        y_pred, mu, logvar = model(x, c, img)
        loss = cvae_loss_function(y_pred, x, mu, logvar, kl_beta=1.0)
        cum_loss += sum(loss).item()
        print(f"batch reconstruction loss: {sum(loss):.4f}")
    cum_loss /= len(test_dataloader)
    print(f"Total reconstruction loss: {cum_loss:.4f}")