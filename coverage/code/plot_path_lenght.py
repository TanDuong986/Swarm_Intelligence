import os
import pandas as pd
import matplotlib.pyplot as plt

# Specify the folder containing the CSV files
folder_path = 'data'

# Get a list of all CSV files in the folder
files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# Create labels from filenames (remove '.csv' and use the filename as the label)
labels = [os.path.splitext(f)[0] for f in files]

# Create a figure
plt.figure(figsize=(10, 6))

# Loop through each file and plot its data
for file, label in zip(files, labels):
    # Read the CSV file
    try:
        df = pd.read_csv(os.path.join(folder_path, file))
        # Plot path_length against iteration
        plt.plot(df['iteration'], df['path_length'], label=label, linewidth=3)
    except FileNotFoundError:
        print(f"File {file} not found. Skipping...")
        continue
    except KeyError:
        print(f"File {file} does not contain required columns. Skipping...")
        continue

# Add labels and title
plt.xlabel('Vòng lặp')
plt.ylabel('Tổng độ dài đường đi (pixel)')
plt.title('So sánh độ dài đường đi giữa các trường hợp khác nhau')
plt.legend()
plt.grid(True)

# Save the plot to a file
plt.savefig('path_length_comparison.png')