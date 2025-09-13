import torch
import numpy as np
from cvae.model.model import CVAE, cvae_loss_function
from tqdm import tqdm
import pandas as pd
from torch.utils.tensorboard import SummaryWriter
from cvae.utils.beta_annealer import BetaAnnealer

import sys
import logging

logging.basicConfig(level=logging.INFO, 
                    format="[%(levelname)s] %(message)s",
                    handlers=[logging.StreamHandler(sys.stdout)])

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
logging.info(f"Using device: {device}")

batch_size = 512

# import data
x_train = pd.read_parquet('cvae/data/data_extended/x_train.parquet')
x_train = x_train.drop(columns=["scenario", "time_step"])

c_train = pd.read_parquet('cvae/data/data_extended/c_train_repeated.parquet')
c_train = c_train.drop(columns=["scenario", "time_step"])

x_val = pd.read_parquet('cvae/data/data_extended/x_validation.parquet')
x_val = x_val.drop(columns=["scenario", "time_step"])

c_val = pd.read_parquet('cvae/data/data_extended/c_validation_repeated.parquet')
c_val = c_val.drop(columns=["scenario", "time_step"])

x_train_t = torch.tensor(x_train.to_numpy(), dtype=torch.float32)
c_train_t = torch.tensor(c_train.to_numpy(), dtype=torch.float32)

x_val_t = torch.tensor(x_val.to_numpy(), dtype=torch.float32)
c_val_t = torch.tensor(c_val.to_numpy(), dtype=torch.float32)

train_dataset = torch.utils.data.TensorDataset(x_train_t, c_train_t)
val_dataset = torch.utils.data.TensorDataset(x_val_t, c_val_t)

train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
val_dataloader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size, shuffle=True)

x_dim = x_train_t.shape[1]
c_dim = c_train_t.shape[1]

logging.info("Data Ready!")

# neural network parameters
# h_Q_dim = 128
# h_P_dim = 128
z_dim = 32  # latent dimension
# X_dim = 6  # input dimension (state)
# c_dim = 133  # conditioning dimension (occ 121, init 6, goal 6)

lr = 1e-3
num_epochs = 5
stall_epochs = 0

kl_beta = 0.0  # KL divergence weight
num_steps = (x_train_t.shape[0] / batch_size) * (num_epochs - stall_epochs)
kl_beta_annealer = BetaAnnealer(beta_start=kl_beta, beta_end=0.5, n_steps=int(num_steps))

# model
model = CVAE(x_dim, c_dim, z_dim).to(device)
optimizer = torch.optim.Adam(model.parameters(), lr=lr)
# lr_scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.995)
# lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer,  step_size=100, gamma=0.8)
# lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=5)

writer = SummaryWriter(
      log_dir=f'cvae/model/runs/lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}'
      )

for epoch in range(num_epochs):
    train_loss, recon_loss, kl_loss = 0.0, 0.0, 0.0
    kl_loss_beta_1 = 0.0
    
    # ----- Training Step -----
    model.train()
    for batch in tqdm(train_dataloader, desc="Training Progress"):
        x, c = batch
        x, c = x.to(device), c.to(device)
        
        y_pred, mu, logvar = model(x, c)
        loss = cvae_loss_function(y_pred, x, mu, logvar, kl_beta=kl_beta)
        optimizer.zero_grad()
        sum(loss).backward()
        optimizer.step()
        
        recon_loss += loss[0].item()
        kl_loss += loss[1].item()
        train_loss += loss[0].item() + loss[1].item()
        
        kl_loss_beta_1 += cvae_loss_function(y_pred, x, mu, logvar, kl_beta=1.0)[1].item()
        
        # logging.info(f"Previous Beta: {kl_beta}")
        if epoch >= stall_epochs:
            kl_beta = kl_beta_annealer.step()
        # logging.info(f"Current Beta: {kl_beta}")
            
    avg_train_recon_loss = recon_loss / len(train_dataloader)
    avg_train_kl_loss = kl_loss / len(train_dataloader)
    avg_train_loss = train_loss / len(train_dataloader)
    
    avg_train_kl_loss_beta_1 = kl_loss_beta_1 / len(train_dataloader)
            
    # ----- Validation Step -----
    model.eval()
    val_loss, recon_loss, kl_loss = 0.0, 0.0, 0.0
    with torch.no_grad():
        for batch in val_dataloader:
            x, c = batch
            x, c = x.to(device), c.to(device)
            
            output, mu, logvar = model(x, c)
            loss = cvae_loss_function(output, x, mu, logvar, kl_beta=1.0)
            
            recon_loss += loss[0].item()
            kl_loss += loss[1].item()
            val_loss += loss[0].item() + loss[1].item()
            
    avg_val_recon_loss = recon_loss / len(val_dataloader)
    avg_val_kl_loss = kl_loss / len(val_dataloader)
    avg_val_loss = val_loss / len(val_dataloader)
    # lr_scheduler.step()
    
    writer.add_scalar('Train_Loss/Full_Loss', avg_train_loss, epoch)
    writer.add_scalar('Train_Loss/Recon_Loss', avg_train_recon_loss, epoch)
    writer.add_scalar('Train_Loss/KL_Loss', avg_train_kl_loss, epoch)
    writer.add_scalar('Train_Loss/KL_Loss_Beta_1.0', avg_train_kl_loss_beta_1, epoch)
    writer.add_scalar('Val_Loss/Full_Loss', avg_val_loss, epoch)
    writer.add_scalar('Val_Loss/Recon_Loss', avg_val_recon_loss, epoch)
    writer.add_scalar('Val_Loss/KL_Loss', avg_val_kl_loss, epoch)
        
    logging.info(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_train_loss:.4f} | Val Loss: {avg_val_loss:.4f}")
      
torch.save(
      model.state_dict(), 
      f'cvae/model/weights/cvae_model_lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}.pth'
      )

logging.info("Training complete. Model saved.")