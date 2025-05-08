import pandas as pd
import matplotlib.pyplot as plt
import os

# Đọc tất cả các tệp CSV trong thư mục "data"
folder_path = 'data/'  # Thay đổi nếu thư mục của bạn ở vị trí khác
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# Lặp qua từng tệp CSV và vẽ biểu đồ
for csv_file in csv_files:
    file_path = os.path.join(folder_path, csv_file)
    data = pd.read_csv(file_path)

    # Vẽ biểu đồ
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Trục đứng 1 cho coverage_ratio
    ax1.set_xlabel('Vòng lặp')
    ax1.set_ylabel('Tỉ lệ (%)', color='tab:red')
    ax1.plot(data['iteration'], data['coverage_ratio'], color='tab:red', label='Tỉ lệ bao phủ', linewidth=2)
    ax1.tick_params(axis='y', labelcolor='tab:red')

    # Thêm legend cho ax1 (coverage_ratio)
    ax1.legend(loc='upper left')

    # Trục đứng 2 cho coverage_cost
    ax2 = ax1.twinx()  # Tạo trục đứng thứ 2 chia sẻ trục hoành
    ax2.set_ylabel('Chi phí bao phủ', color='tab:blue')
    ax2.plot(data['iteration'], data['coverage_cost'], color='tab:blue', label='Chi phí bao phủ', linewidth=2)
    ax2.tick_params(axis='y', labelcolor='tab:blue')

    # Thêm legend cho ax2 (coverage_cost)
    ax2.legend(loc='upper right')

    # Thêm tiêu đề và hiển thị biểu đồ
    plt.title(f'Biểu đồ chi phí và tỉ lệ bao phủ theo vòng lặp')
    fig.tight_layout()  # Đảm bảo các nhãn không bị trùng

    # Lưu hình ảnh vào file với tên tệp CSV
    save_path = os.path.join(folder_path, f'cm_{csv_file}.png')
    plt.savefig(save_path, dpi=300)  # Lưu với độ phân giải cao

    # Hiển thị biểu đồ
    plt.show()
