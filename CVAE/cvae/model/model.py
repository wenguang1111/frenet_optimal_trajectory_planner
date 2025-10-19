import torch
import torch.nn as nn
import torch.nn.functional as F

class CVAE(nn.Module):
    def __init__(self, X_dim, c_dim, z_dim, h_Q_dim=512, h_P_dim=512):
        super(CVAE, self).__init__()
        
        # Encoder (Q network)
        self.encoder_fc_q1 = nn.Linear(X_dim + c_dim, h_Q_dim)
        self.encoder_fc_q2 = nn.Linear(h_Q_dim, h_Q_dim // 2)
        self.encoder_fc_mu = nn.Linear(h_Q_dim // 2, z_dim)
        self.encoder_fc_logvar = nn.Linear(h_Q_dim // 2, z_dim)
        
        # Decoder (P network)
        self.decoder_fc_p1 = nn.Linear(z_dim + c_dim, h_P_dim // 2)
        self.decoder_fc_p2 = nn.Linear(h_P_dim // 2, h_P_dim // 4)
        self.decoder_fc_p3 = nn.Linear(h_P_dim // 4, h_P_dim // 8)
        self.decoder_fc_out = nn.Linear(h_P_dim // 8, X_dim)

    def encode(self, x, c):
        xc = torch.cat([x, c], dim=1)
        h = F.relu(self.encoder_fc_q1(xc))
        # h = F.dropout(h, p=0.5, training=self.training)
        h = F.relu(self.encoder_fc_q2(h))
        mu = self.encoder_fc_mu(h)
        logvar = self.encoder_fc_logvar(h)
        return mu, logvar

    def reparameterize(self, mu, logvar):
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return mu + eps * std

    def decode(self, z, c):
        zc = torch.cat([z, c], dim=1)
        h = F.relu(self.decoder_fc_p1(zc))
        # h = F.dropout(h, p=0.5)
        h = F.relu(self.decoder_fc_p2(h))
        h = F.relu(self.decoder_fc_p3(h))
        return self.decoder_fc_out(h)

    def forward(self, x, c):
        mu, logvar = self.encode(x, c)
        z = self.reparameterize(mu, logvar)
        y = self.decode(z, c)
        return y, mu, logvar


def cvae_loss_function(y_pred, y_true, mu, logvar, kl_beta=1e-4):
    # incorporate the range of d, t, v
    recon_loss = F.mse_loss(y_pred, y_true)

    # sum over latent dim
    kl_loss = -0.5 * torch.sum(1 + logvar - mu.pow(2) - logvar.exp(), dim=1)
    # mean over batch
    kl_loss = kl_loss.mean()
    # print(f"KL Loss: {kl_loss}, Recon Loss: {recon_loss}")
    return recon_loss, kl_beta * kl_loss


if __name__ == "__main__":
    # Example usage and hook for layer output shapes
    X_dim, c_dim, z_dim = 3, 518, 32
    model = CVAE(X_dim=3, c_dim=518, z_dim=32)

    # Hook function using the layer's variable name
    def print_shapes(name, module, input, output):
        location = "Encoder" if "encoder" in name else "Decoder" if "decoder" in name else "Other"
        print(f"{name} ({location}):")
        print(f"  Input: {[i.shape for i in input]}")
        print(f"  Output: {output.shape if isinstance(output, torch.Tensor) else [o.shape for o in output]}")
        if location == "Encoder" and output.shape[1] == z_dim:
            print(f"  **Latent z dimension: {output.shape}**")

    # Register hooks
    hooks = []
    for name, layer in model.named_modules():
        if isinstance(layer, nn.Linear):
            # Use lambda to pass the layer name
            hooks.append(layer.register_forward_hook(lambda m, inp, out, n=name: print_shapes(n, m, inp, out)))

    # Dummy input
    x = torch.randn(1, X_dim)
    c = torch.randn(1, c_dim)
    x_recon = model(x, c)

    # Remove hooks
    for h in hooks:
        h.remove()