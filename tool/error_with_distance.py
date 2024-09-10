import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Step 1: Load the data from the CSV file
file_path = 'build/variable_float.csv'
df = pd.read_csv(file_path)

# Step 2: Create intervals of 1 meter
df['x_interval'] = df['x'].apply(lambda x: int(np.floor(x)))

# Step 3: Group by the interval and calculate the mean of zelt_x
error_analysis = df.groupby('x_interval')['zelt_x'].mean().reset_index()

# Rename the columns for clarity
error_analysis.columns = ['x_interval', 'average_zelt_x']

# Step 4: Save the result to a new CSV file
# output_file_path = '/mnt/data/error_analysis.csv'
# error_analysis.to_csv(output_file_path, index=False)

# Step 5: Plot the data
plt.figure(figsize=(10, 6))
plt.plot(error_analysis['x_interval'], error_analysis['average_zelt_x'], marker='o', linestyle='-', color='k')
plt.title('Average Error in X Position vs X Interval')
plt.xlabel('X Interval (meters)')
plt.ylabel('Average Error (zelt_x)')
plt.grid(True)
# Show the plot
plt.show()

# # Save the plot as an image file
# plot_output_path = '/mnt/data/average_error_plot.png'
# plt.savefig(plot_output_path)