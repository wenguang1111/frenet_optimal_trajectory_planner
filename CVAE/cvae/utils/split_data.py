import os
import shutil
import pandas as pd
from sklearn.model_selection import train_test_split

data_dir = "cvae/data/data_v2/"

# Load full dataset
c_df = pd.read_parquet(data_dir + "conditioned_vars.parquet")
x_df = pd.read_parquet(data_dir + "sampled_vars.parquet")

scenarios = x_df["scenario"].unique()

# 80, 10, 10 split
train_sc, temp_sc = train_test_split(scenarios, test_size=0.2)
val_sc, test_sc = train_test_split(temp_sc, test_size=0.5)

x_train = x_df[x_df["scenario"].isin(train_sc)]
x_val = x_df[x_df["scenario"].isin(val_sc)]
x_test = x_df[x_df["scenario"].isin(test_sc)]

c_train = c_df[c_df["scenario"].isin(train_sc)]
c_val = c_df[c_df["scenario"].isin(val_sc)]
c_test = c_df[c_df["scenario"].isin(test_sc)]

x_train.to_parquet(data_dir + "x_train.parquet", index=False)
x_val.to_parquet(data_dir + "x_validation.parquet", index=False)
x_test.to_parquet(data_dir + "x_test.parquet", index=False)

c_train.to_parquet(data_dir + "c_train.parquet", index=False)
c_val.to_parquet(data_dir + "c_validation.parquet", index=False)
c_test.to_parquet(data_dir + "c_test.parquet", index=False)

for sc in train_sc:
    src = os.path.join(data_dir + "scenarios_imgs/", sc)
    dst = os.path.join(data_dir + "scenarios_imgs/split/train/", sc)
    if os.path.exists(src):
        shutil.copytree(src, dst, dirs_exist_ok=True)
        
    print(f"Copied training scenario: {sc}")

for sc in val_sc:
    src = os.path.join(data_dir + "scenarios_imgs/", sc)
    dst = os.path.join(data_dir + "scenarios_imgs/split/validation/", sc)
    if os.path.exists(src):
        shutil.copytree(src, dst, dirs_exist_ok=True)
        
    print(f"Copied validation scenario: {sc}")

for sc in test_sc:
    src = os.path.join(data_dir + "scenarios_imgs/", sc)
    dst = os.path.join(data_dir + "scenarios_imgs/split/test/", sc)
    if os.path.exists(src):
        shutil.copytree(src, dst, dirs_exist_ok=True)
        
    print(f"Copied test scenario: {sc}")