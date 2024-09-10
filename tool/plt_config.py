import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.size': 14,         # Default font size for text
    'font.family': 'lucida',  # Set the font family to serif (like LaTeX)
    # 'font.serif': ['Times New Roman'],  # Use a specific serif font
    'axes.titlesize': 18,    # Font size for plot titles
    'axes.labelsize': 16,    # Font size for axis labels
    'legend.fontsize': 16,   # Font size for legend
    'xtick.labelsize': 16,   # Font size for x-axis tick labels
    'ytick.labelsize': 16,   # Font size for y-axis tick labels
    'text.usetex': True      # Use LaTeX to render text (optional, requires LaTeX installed)
})