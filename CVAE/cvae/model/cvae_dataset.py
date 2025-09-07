import torch
from torch.utils.data import Dataset, DataLoader
import pandas as pd


class CVAEDataset(Dataset):
    def __init__(self, x_df, c_df):

        merged = pd.merge(x_df, c_df, on=["scenario", "time_step"], how="inner")
        print(merged.head())
        cols_to_drop = [merged.columns[i] for i in range(2, 5)]
        new_c = merged.drop(columns=cols_to_drop)
        print(new_c.head())
        new_c.to_csv("cvae/data/data_extended/test_conditions_repeated.csv", index=False)
        print("Done")
        # self.samples = []
        # x_cols = [col for col in x_df.columns if col not in ["scenario", "time_step"]]
        # c_cols = [col for col in c_df.columns if col not in ["scenario", "time_step"]]

        # for row in merged.itertuples(index=False):
        #     x_tensor = torch.tensor([getattr(row, col) for col in x_cols], dtype=torch.float32)
        #     c_tensor = torch.tensor([getattr(row, col) for col in c_cols], dtype=torch.float32)
        #     self.samples.append((x_tensor, c_tensor))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        return self.samples[idx]

if __name__ == "__main__":
    x = pd.read_csv("cvae/data/data_extended/x_test.csv")
    x = x.drop(["Unnamed: 0"], axis=1)
   
    c = pd.read_csv("cvae/data/data_extended/c_test.csv")
    c = c.drop(["Unnamed: 0"], axis=1)
    
    dataset = CVAEDataset(x_df=x, c_df=c)
    loader = DataLoader(dataset, batch_size=2, shuffle=True)

    for x, c in loader:
        print(x.shape, c.shape)
