import pandas as pd

data_dir = "cvae/data/data_extended/"

df = pd.read_csv(data_dir + "x_train.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "x_train.parquet", index=False)

df = pd.read_csv(data_dir + "x_test.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "x_test.parquet", index=False)

df = pd.read_csv(data_dir + "x_validation.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "x_validation.parquet", index=False)

df = pd.read_csv(data_dir + "c_train.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "c_train.parquet", index=False)

df = pd.read_csv(data_dir + "c_test.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "c_test.parquet", index=False)

df = pd.read_csv(data_dir + "c_validation.csv")
df = df.drop(columns=["Unnamed: 0"])
df.to_parquet(data_dir + "c_validation.parquet", index=False)

x_train = pd.read_parquet(data_dir + "x_train.parquet")
x_test = pd.read_parquet(data_dir + "x_test.parquet")
x_validation = pd.read_parquet(data_dir + "x_validation.parquet")

c_train = pd.read_parquet(data_dir + "c_train.parquet")
c_test = pd.read_parquet(data_dir + "c_test.parquet")
c_validation = pd.read_parquet(data_dir + "c_validation.parquet")

train_merged = pd.merge(x_train, c_train, on=["scenario", "time_step"], how="inner")
validation_merged = pd.merge(x_validation, c_validation, on=["scenario", "time_step"], how="inner")
test_merged = pd.merge(x_test, c_test, on=["scenario", "time_step"], how="inner")

cols_to_drop = [train_merged.columns[i] for i in range(2, 5)]

c_train = train_merged.drop(columns=cols_to_drop)
c_validation = validation_merged.drop(columns=cols_to_drop)
c_test = test_merged.drop(columns=cols_to_drop)

c_train.to_parquet(data_dir + "c_train_repeated.parquet", index=False)
c_test.to_parquet(data_dir + "c_test_repeated.parquet", index=False)
c_validation.to_parquet(data_dir + "c_validation_repeated.parquet", index=False)

