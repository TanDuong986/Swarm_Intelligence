import matplotlib.pyplot as plt
import numpy as np

# Tạo grid map 100x100
grid_size = 100
grid_map = np.zeros((grid_size, grid_size))

# Thêm một số vật cản (các ô giá trị 1)
grid_map[30:40, 30:40] = 1  # Vật cản hình vuông
grid_map[60:70, 60:70] = 1  # Vật cản khác

# Vẽ grid map
plt.imshow(grid_map, cmap='Greys', origin='lower')
plt.colorbar()
# plt.axis('off')
plt.show()
