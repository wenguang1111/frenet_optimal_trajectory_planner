import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Set global font properties
plt.rcParams.update({
    'font.size': 14,         # Default font size for text
    'font.family': 'serif',  # Set the font family to serif (like LaTeX)
    'font.serif': ['Times New Roman'],  # Use a specific serif font
    'axes.titlesize': 18,    # Font size for plot titles
    'axes.labelsize': 16,    # Font size for axis labels
    'legend.fontsize': 14,   # Font size for legend
    'xtick.labelsize': 12,   # Font size for x-axis tick labels
    'ytick.labelsize': 12,   # Font size for y-axis tick labels
    'text.usetex': True      # Use LaTeX to render text (optional, requires LaTeX installed)
})

# Read the CSV file
df = pd.read_csv('build/variable_float.csv')

# Extract relevant columns
x = df['x']
y = df['y']
time = df['time']

# Determine where each new trajectory starts
trajectory_starts = np.where(time == 0)[0]
trajectory_ends = []

# Loop through the indices of zeros
for i in trajectory_starts:
    if i > 0:  # To avoid indexing before the first element
        trajectory_ends.append(i-1)
trajectory_ends.append(trajectory_starts[-1]+trajectory_ends[0]-trajectory_starts[0])
# Initialize plot
plt.figure(figsize=(15, 8))

# Iterate over each trajectory
for i in range(len(trajectory_starts)):
    # Determine the start and end indices of the trajectory
    start_idx = trajectory_starts[i]
    end_idx = trajectory_ends[i]
    # Get the corresponding trajectory data
    x_trajectory = x[start_idx:end_idx]
    y_trajectory = y[start_idx:end_idx]
    # Plot the trajectory
    plt.plot(x_trajectory, y_trajectory)

plt.plot(df['WP_x'], df['WP_y'], 'k-', label='Way Point')
# Adding labels and title
plt.xlabel('X[m]')
plt.ylabel('Y[m]')

# Add legend
# plt.legend()

# Show the plot
plt.show()