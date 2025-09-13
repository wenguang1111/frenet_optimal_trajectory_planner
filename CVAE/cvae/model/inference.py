import torch
import numpy as np
from cvae.model.model import CVAE, cvae_loss_function
import time
import pandas as pd

# neural network parameters
batch_size = 512
# h_Q_dim = 512
# h_P_dim = 512
z_dim = 32  # latent dimension

# import data
x_test = pd.read_parquet('cvae/data/data_extended/x_test.parquet')
x_test = x_test.drop(columns=["scenario", "time_step"])

c_test = pd.read_parquet('cvae/data/data_extended/c_test_repeated.parquet')
c_test = c_test.drop(columns=["scenario", "time_step"])

x_test_t = torch.tensor(x_test.to_numpy(), dtype=torch.float32)
c_test_t = torch.tensor(c_test.to_numpy(), dtype=torch.float32)

test_dataset = torch.utils.data.TensorDataset(x_test_t, c_test_t)
test_dataloader = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size, shuffle=True)

x_dim = x_test_t.shape[1]
c_dim = c_test_t.shape[1]

# Load the model
model = CVAE(x_dim, c_dim, z_dim)
model.load_state_dict(torch.load('cvae/model/weights/cvae_model_lr_0.001_batch_512_epochs_5_zdim_32.pth'))
model.eval()

# s_time = time.time()
with torch.inference_mode():
    loss = 0.0
    for batch in test_dataloader:
        x, c = batch
        y_pred, mu, logvar = model(x, c)
        # print(y_pred, x)
        recon_loss, kl_loss = cvae_loss_function(y_pred, x, mu, logvar)
        batch_loss = recon_loss.item() + kl_loss.item()
        loss += batch_loss
        print(f"batch reconstruction loss: {batch_loss:.4f}")
    loss /= len(test_dataloader)
    print(f"Total reconstruction loss: {loss:.4f}")