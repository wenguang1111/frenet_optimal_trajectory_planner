import pandas as pd
import matplotlib.pyplot as plt
import math
import plt_config

file_path = 'tool/Summary_T5_polynomial.csv'  # replace with your actual file path
df_poly = pd.read_csv(file_path)
file_path2 = 'tool/Summary_D50_DifferentFractionBit.csv'  # replace with your actual file path
df_cubic = pd.read_csv(file_path2)
file_path3 = 'tool/Summary_Frenet.csv'  # replace with your actual file path
df_frenet = pd.read_csv(file_path3)
plt.figure(figsize=(10, 6))

# cubic spline analyze
plt.plot(df_poly['Fraction_Bit'], df_poly['Max_x'], color='orange', marker='o', linestyle='-',label=r'$max(d)$')  # red color, circle marker
plt.plot(df_cubic['BitWidth'], df_cubic['max_x'], color='green', marker='o', linestyle='-', label=r'$max(x)$')  # red color, circle marker
plt.plot(df_cubic['BitWidth'], df_cubic['max_y'],color='green', marker='s', linestyle='-', label=r'$max(y)$')  # red color, circle marker
plt.plot(df_frenet['BitWidth'], df_frenet['max_x'], color='blue', marker='o', linestyle='-', label=r'$max(x)$')  # red color, circle marker
plt.plot(df_frenet['BitWidth'], df_frenet['max_y'],color='blue', marker='s', linestyle='-', label=r'$max(y)$')  # red color, circle marker

# Set logarithmic scale for y-axis (log base 2)
plt.yscale('log', base=10)

plt.xlabel('Bit Width [bit]')
plt.ylabel('Precision Loss [m]')
# y_ticks = [0.001 * 2**i for i in range(5)]  # Adjust the range as needed
# plt.yticks(y_ticks)

plt.legend()
plt.grid(True)
plt.show()
