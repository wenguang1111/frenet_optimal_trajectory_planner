import torch
import cv2

class MaskBackground(torch.nn.Module):

    def __init__(self):
        super().__init__()

    def forward(self, img):
        
        is_white = (torch.abs(img[0, :, :] - 1.0) <= 0.0) & \
           (torch.abs(img[1, :, :] - 1.0) <= 0.0) & \
           (torch.abs(img[2, :, :] - 1.0) <= 0.0)
        
        # Create mask: True where pixel is not white
        mask = ~is_white
        mask = mask.unsqueeze(0)

        masked_img = mask.float() * img
        
        # validate by saving a random masked image
        # img_to_save = (masked_img * 255).byte().cpu().numpy()
        # img_to_save = img_to_save.transpose(1, 2, 0)
        # img_to_save = cv2.cvtColor(img_to_save, cv2.COLOR_RGB2BGR)
        # random_id = torch.randint(0, 1000000, (1,)).item()
        # cv2.imwrite(f"/dss/dsshome1/07/di97xub/frenet_optimal_trajectory_planner/CVAE/restore/{random_id}.png", img_to_save)
        
        return masked_img
    