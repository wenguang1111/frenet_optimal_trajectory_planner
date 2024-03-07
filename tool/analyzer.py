import pandas as pd
import matplotlib.pyplot as plt

file_path = 'variable_double.csv'
data = pd.read_csv(file_path)

column_stats = {}

for column in data.columns:
    max_value = data[column].max()
    min_value = data[column].min()
    std_dev = data[column].std()
    num_cal = data[column].count()
    column_stats[column] = {'max_value': max_value, 'min_value': min_value, 'std_dev': std_dev, 'num_cal': num_cal}

# Find the maximum number of rows among all columns
max_num_cal = max(stats['num_cal'] for stats in column_stats.values())

for column, stats in column_stats.items():
    print(f"variable： {column}:")
    print("max_value:", stats['max_value'])
    print("min_value:", stats['min_value'])
    print("std_dev:", stats['std_dev'])
    print("num_cal:", stats['num_cal'])
    print("num_cal/max(num_call[i]):", stats['num_cal']/max_num_cal*100)
    print("\n")

# values = data['Car::getOutline()::head_l[i]']
# plt.hist(values, bins=100, edgecolor='black')
# plt.title(f"Histogram for {column}")
# plt.xlabel(column)
# plt.ylabel("Frequency")
# plt.show()