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
from cvae.model.normalizer import Normalizer
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

log_every = 10  # batches

# parquet paths
train_imgs_root = 'cvae/data/data_v2/train/imgs/'
targets_train_path = 'cvae/data/data_v2/train/x_train.parquet'
conditions_train_path = 'cvae/data/data_v2/train/c_train.parquet'

val_imgs_root = 'cvae/data/data_v2/val/imgs/'
targets_val_path = 'cvae/data/data_v2/val/x_validation.parquet'
conditions_val_path = 'cvae/data/data_v2/val/c_validation.parquet'

data_normalizer = Normalizer()

imgs_transforms = transforms.Compose([
    transforms.Resize((img_dim, img_dim)),
    transforms.ToTensor(),  # Converts to [0, 1] (normalized)
    MaskBackground()
])

# prepare datasets and dataloaders
train_dataset = CVAEDataset(
    targets_path=targets_train_path,
    conditions_path=conditions_train_path,
    image_root=train_imgs_root,
    image_transform=imgs_transforms,
    data_normalizer=data_normalizer,
    num_workers=num_workers
)

val_dataset = CVAEDataset(
    targets_path=targets_val_path,
    conditions_path=conditions_val_path,
    image_root=val_imgs_root,
    image_transform=imgs_transforms,
    data_normalizer=data_normalizer,
    num_workers=num_workers
)

train_dataloader = torch.utils.data.DataLoader(train_dataset, 
                                               batch_size=batch_size,
                                               pin_memory=True,
                                               num_workers=num_workers,
                                               shuffle=True)
val_dataloader = torch.utils.data.DataLoader(val_dataset, 
                                             batch_size=batch_size,
                                             pin_memory=True,
                                             num_workers=num_workers,
                                             shuffle=True)

logging.info("Data Ready!")

lr = 5e-6
num_epochs = 20
stall_epochs = 0

kl_beta = 0.0  # KL divergence weight
# num_steps = (x_train_t.shape[0] / batch_size) * (num_epochs - stall_epochs)
num_steps = (len(train_dataset) / batch_size) * (num_epochs - stall_epochs)
kl_beta_annealer = BetaAnnealer(beta_start=kl_beta, beta_end=1.0, n_steps=int(num_steps))

# model
model = CVAE(x_dim, c_dim, z_dim, ).to(device)

# optimizer = torch.optim.Adam(model.parameters(), lr=lr)
optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-2)
# lr_scheduler = torch.optim.lr_scheduler.ExponentialLR(optimizer, gamma=0.995)
# lr_scheduler = torch.optim.lr_scheduler.StepLR(optimizer,  step_size=100, gamma=0.8)
lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(optimizer, 
                                                          mode='min',
                                                          factor=0.5,
                                                          patience=3,
                                                          min_lr=1e-7)

writer = SummaryWriter(
      log_dir=f'cvae/model/runs/lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}'
      )

# start_data = torch.cuda.Event(enable_timing=True)
# end_data = torch.cuda.Event(enable_timing=True)
# start_forward = torch.cuda.Event(enable_timing=True)
# end_forward = torch.cuda.Event(enable_timing=True)
# start_backward = torch.cuda.Event(enable_timing=True)
# end_backward = torch.cuda.Event(enable_timing=True)

# Training loop
for epoch in range(num_epochs):
    train_loss, recon_loss, kl_loss = 0.0, 0.0, 0.0
    kl_loss_beta_1 = 0.0
    
    # ----- Training Step -----
    model.train()
    for i, batch in enumerate(train_dataloader):
        # start_data.record()
        x, c, img = batch
        x, c, img = x.to(device, non_blocking=True), c.to(device, non_blocking=True), img.to(device, non_blocking=True)
        # end_data.record()
        
        # start_forward.record()
        y_pred, mu, logvar = model(x, c, img)
        loss = cvae_loss_function(y_pred, x, mu, logvar, kl_beta=kl_beta)
        # end_forward.record()
        
        # start_backward.record()
        optimizer.zero_grad(set_to_none=True)  # clear gradients more effieciently (faster and saves memory)
        sum(loss).backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        optimizer.step()
        # scheduler.step()
        # end_backward.record()
        
        recon_loss += loss[0].item()
        kl_loss += loss[1].item()
        train_loss += loss[0].item() + loss[1].item()
        
        kl_loss_beta_1 += cvae_loss_function(y_pred, x, mu, logvar, kl_beta=1.0)[1].item()
        
        # Wait for GPU to finish
        # torch.cuda.synchronize()
        
        # logging.info(f"Data move: {start_data.elapsed_time(end_data)/1000:.3f}s | "
        #     f"Forward: {start_forward.elapsed_time(end_forward)/1000:.3f}s | "
        #     f"Backward: {start_backward.elapsed_time(end_backward)/1000:.3f}s")
        if (i + 1) % log_every == 0:
            logging.info(f"Epoch {epoch+1} | "
                         f"Batch {i+1}/{len(train_dataloader)} |"
                         f"Recon Loss: {loss[0].item():.4f} | "
                         f"KL Loss: {loss[1].item():.4f} | "
                         f"KL Beta: {kl_beta:.4f}")
        
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
            x, c, img = batch
            x, c, img = x.to(device, non_blocking=True), c.to(device, non_blocking=True), img.to(device, non_blocking=True)
            
            output, mu, logvar = model(x, c, img)
            loss = cvae_loss_function(output, x, mu, logvar, kl_beta=1.0)
            
            recon_loss += loss[0].item()
            kl_loss += loss[1].item()
            val_loss += loss[0].item() + loss[1].item()
            
    avg_val_recon_loss = recon_loss / len(val_dataloader)
    avg_val_kl_loss = kl_loss / len(val_dataloader)
    avg_val_loss = val_loss / len(val_dataloader)
    
    lr_scheduler.step()
    current_lr = optimizer.param_groups[0]['lr']
    
    writer.add_scalar('Train_Loss/Full_Loss', avg_train_loss, epoch)
    writer.add_scalar('Train_Loss/Recon_Loss', avg_train_recon_loss, epoch)
    writer.add_scalar('Train_Loss/KL_Loss', avg_train_kl_loss, epoch)
    writer.add_scalar('Train_Loss/KL_Loss_Beta_1.0', avg_train_kl_loss_beta_1, epoch)
    writer.add_scalar('Val_Loss/Full_Loss', avg_val_loss, epoch)
    writer.add_scalar('Val_Loss/Recon_Loss', avg_val_recon_loss, epoch)
    writer.add_scalar('Val_Loss/KL_Loss_Beta_1.0', avg_val_kl_loss, epoch)
    writer.add_scalar('Learning_Rate', current_lr, epoch)
        
    logging.info(f"Epoch {epoch+1}/{num_epochs} | "
                f"Train Loss: {avg_train_loss:.4f} | "
                f"Val Loss: {avg_val_loss:.4f} | "
                f"LR: {current_lr:.2e}")
      
torch.save(
      model.state_dict(), 
      f'cvae/model/weights/cvae_model_lr_{lr}_batch_{batch_size}_epochs_{num_epochs}_zdim_{z_dim}.pth'
      )

logging.info("Training complete. Model saved.")