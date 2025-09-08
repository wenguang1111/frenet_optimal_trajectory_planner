# broadcast the conditions on the x variables and save the broadcasted conditions
import pandas as pd

x_train = pd.read_csv("cvae/data/data_extended/x_train.csv")
x_test = pd.read_csv("cvae/data/data_extended/x_test.csv")
x_validation = pd.read_csv("cvae/data/data_extended/x_validation.csv")

c_train = pd.read_csv("cvae/data/data_extended/c_train.csv")
c_test = pd.read_csv("cvae/data/data_extended/c_test.csv")
c_validation = pd.read_csv("cvae/data/data_extended/c_validation.csv")

train_merged = pd.merge(x_train, c_train, on=["scenario", "time_step"], how="inner")
validation_merged = pd.merge(x_validation, c_validation, on=["scenario", "time_step"], how="inner")
test_merged = pd.merge(x_test, c_test, on=["scenario", "time_step"], how="inner")

cols_to_drop = [train_merged.columns[i] for i in range(2, 5)]

c_train = train_merged.drop(columns=cols_to_drop)
c_validation = validation_merged.drop(columns=cols_to_drop)
c_test = test_merged.drop(columns=cols_to_drop)

c_train.to_parquet("cvae/data/data_extended/c_train_repeated.parquet", index=False)
c_test.to_parquet("cvae/data/data_extended/c_test_repeated.parquet", index=false)
c_validation.to_parquet("cvae/data/data_extended/c_validation_repeated.parquet", index=False)

