import random

# Kích thước bản đồ
grid_size = 100
grid_map = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# Thêm vật cản ngẫu nhiên
for i in range(grid_size):
    for j in range(grid_size):
        if random.random() < 0.3:  # 30% tỷ lệ vật cản
            grid_map[i][j] = 1  # 1 là vật cản

# Đảm bảo có một đường đi khả thi từ (0, 0) đến (grid_size-1, grid_size-1)
grid_map[0][0] = 0
grid_map[grid_size-1][grid_size-1] = 0

# In ra bản đồ
for row in grid_map:
    print(" ".join(map(str, row)))

