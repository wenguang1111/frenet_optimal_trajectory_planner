import pandas as pd
from sklearn.model_selection import train_test_split

# Load full dataset
c_df = pd.read_csv("cvae/data/data_extended/conditioned_vars_w_imgs.csv")
x_df = pd.read_csv("cvae/data/data_extended/sampled_vars.csv")

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

x_train.to_csv("cvae/data/data_extended/x_train.csv", index=False)
x_val.to_csv("cvae/data/data_extended/x_validation.csv", index=False)
x_test.to_csv("cvae/data/data_extended/x_test.csv", index=False)

c_train.to_csv("cvae/data/data_extended/c_train.csv", index=False)
c_val.to_csv("cvae/data/data_extended/c_validation.csv", index=False)
c_test.to_csv("cvae/data/data_extended/c_test.csv", index=False)
