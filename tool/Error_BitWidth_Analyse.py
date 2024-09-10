import pandas as pd
import matplotlib.pyplot as plt
import plt_config


file_path = 'tool/Summary_Frenet.csv'  # replace with your actual file path
df = pd.read_csv(file_path)
plt.figure(figsize=(10, 6))

# cubic spline analyze
plt.plot(df['BitWidth'], df['a_x'], 'ks--', label=r'$\bar{x}$')  # red color, circle marker
plt.plot(df['BitWidth'], df['a_y'], 'ko--', label=r'$\bar{y}$') # red color, square marker

# # polynomial analyze
# plt.plot(df['Fraction_Bit'], df['Average_x'], 'ks-', label=r'$\bar{d}$')  # red color, circle marker
# plt.plot(df['Fraction_Bit'], df['Max_x'], 'ko-', label=r'$\max(d)$') # red color, square marker

# Plot max_x and max_y
plt.plot(df['BitWidth'], df['max_x'], 'ks-', label=r'$max(x)$')  # blue color, circle marker
plt.plot(df['BitWidth'], df['max_y'], 'ko-', label=r'$max(y)$') # blue color, square marker


plt.xlabel('Bit Width [bit]')
plt.ylabel('Precision Loss [m]')
# y_ticks = [0.001 * 2**i for i in range(5)]  # Adjust the range as needed
# plt.yticks(y_ticks)

plt.legend()
plt.grid(True)
plt.show()
