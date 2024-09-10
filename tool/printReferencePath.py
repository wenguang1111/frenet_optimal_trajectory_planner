import pandas as pd
import matplotlib.pyplot as plt
import plt_config

# Load the CSV file
file_path = 'build/variable_float.csv'
df = pd.read_csv(file_path)

# Plot the data
plt.figure(figsize=(10, 6))

# Plot a_x and a_y
plt.plot(df['WP_x'], df['WP_y'], 'k-', label='Way Point')
point_x =[0,10,20,30,40,50]
point_y = [10,0,-10,-10,0,10]
# Labeling the axes
plt.xlabel('x [m]')
plt.ylabel('y [m]')

# Show legend
# plt.legend()

# Show the plot
plt.grid(True)
plt.show()
