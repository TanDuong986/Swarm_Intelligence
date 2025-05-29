import numpy as np
import matplotlib.pyplot as plt

# Kích thước ảnh (pixel)
w, h = 200, 200

# Tâm của "quầng sáng" (tọa độ pixel)
x0, y0 = 100, 120
r = 30  # bán kính làm mờ

# Tạo lưới pixel
y_idx, x_idx = np.indices((h, w))  # (h, w) theo đúng thứ tự (row, col)

# Tính giá trị hàm tại mỗi điểm
img = np.exp(-((x_idx - x0)**2 + (y_idx - y0)**2) / (2 * r**2))

# Hiển thị ảnh xám
plt.figure(figsize=(6,6))
plt.imshow(img, cmap='gray', origin='upper')
plt.colorbar(label='Giá trị hàm e mũ -')
plt.title('Gaussian "inflation" quanh tâm ({}, {}) bán kính {}'.format(x0, y0, r))
plt.scatter([x0], [y0], color='red', marker='x', label='Tâm')
plt.legend()
plt.show()
