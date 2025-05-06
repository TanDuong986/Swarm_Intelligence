import pandas as pd
import numpy as np
import os
import glob
import matplotlib.pyplot as plt

def process_csv_files(folder_path):
    # Tìm tất cả các tệp CSV trong thư mục
    csv_files = glob.glob(os.path.join(folder_path, "*.csv"))
    
    # Đảm bảo ít nhất có một tệp CSV
    if len(csv_files) == 0:
        print("Không tìm thấy tệp CSV nào trong thư mục.")
        return None, None
    
    # Đọc tất cả các tệp CSV vào một danh sách
    all_data = [pd.read_csv(file_path) for file_path in csv_files]
    
    # Kiểm tra dữ liệu đầu tiên để đảm bảo đã đọc đúng
    print("Đã đọc {} tệp CSV.".format(len(all_data)))
    
    # Tạo mảng cho 'Order' từ tất cả các tệp
    order_data = np.array([data['Order'].values for data in all_data])

    # Tính trung bình và độ lệch chuẩn cho 'Order' tại mỗi bước
    order_mean = np.nanmean(order_data, axis=0)
    order_std = np.nanstd(order_data, axis=0)

    # Tạo mảng cho 'Entropy' từ tất cả các tệp
    entropy_data = np.array([data['Entropy'].values for data in all_data])

    # Tính trung bình và độ lệch chuẩn cho 'Entropy' tại mỗi bước
    entropy_mean = np.nanmean(entropy_data, axis=0)
    entropy_std = np.nanstd(entropy_data, axis=0)

    return order_mean, order_std, entropy_mean, entropy_std

# Đường dẫn đến thư mục chứa các tệp CSV của bạn
folder_path = "obstacle_cheo"

# Gọi hàm xử lý
order_mean, order_std, entropy_mean, entropy_std = process_csv_files(folder_path)
time_steps = np.arange(len(order_mean))

# Kiểm tra và in kết quả (5 phần tử đầu tiên)

plt.figure(figsize=(8, 6))
plt.plot(time_steps, order_mean, label='Giá trị order theo thời gian', color='orange')
plt.fill_between(time_steps, order_mean - order_std, order_mean + order_std, alpha=0.3, color='orange')
plt.ylim(0, 1.5)
plt.xlabel("Bước thời gian (t)")
plt.ylabel("Giá trị order")
plt.title("Biểu đồ giá trị order theo thời gian")
plt.grid(True)
plt.legend(loc='upper left')
plt.tight_layout()
plt.savefig("order_parameter_obs.png", dpi=300)
plt.show()

# Vẽ Normalized Entropy với vùng shading (fill_between)
plt.figure(figsize=(8, 6))
plt.plot(time_steps, entropy_mean, label='Giá trị entropy ', color='red')
plt.fill_between(time_steps, entropy_mean - entropy_std, entropy_mean + entropy_std, alpha=0.3, color='red')
plt.ylim(0, 1.5)
plt.xlabel("Bước thời gian (t)")
plt.ylabel("Giá trị entropy")
plt.title("Giá trị entropy theo thời gian")
plt.grid(True)
plt.legend(loc='upper left')
plt.tight_layout()
plt.savefig("entropy_obs.png", dpi=300)
plt.show()
