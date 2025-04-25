import pygame
import random

# Hàm tạo màu ngẫu nhiên
def random_color():
    return random.choice([[255, 0, 255], [255, 255, 255]])

# Hàm chính vẽ grid map
def draw_maze(maze, grid_size, screen):
    # Kích thước ô trong grid
    cell_size = 600 // grid_size

    # Vẽ các ô có màu ngẫu nhiên (màu vật cản)
    for i in range(grid_size):
        for j in range(grid_size):
            color = [255, 255, 255] if maze[i][j] == 0 else [255, 0, 255]  # Trắng cho ô trống, hồng cho vật cản
            pygame.draw.rect(screen, color, (i * cell_size, j * cell_size, cell_size, cell_size))

            # Vẽ các đường viền ô màu đen
            pygame.draw.rect(screen, (0, 0, 0), (i * cell_size, j * cell_size, cell_size, cell_size), 1)

# Hàm tạo mê cung dễ
def generate_easy_maze(grid_size):
    maze = [[0 for _ in range(grid_size)] for _ in range(grid_size)]
    for i in range(grid_size):
        for j in range(grid_size):
            if random.random() < 0.2:  # 20% vật cản
                maze[i][j] = 1
    maze[0][0] = 0  # Đảm bảo điểm bắt đầu là mở
    maze[grid_size-1][grid_size-1] = 0  # Đảm bảo điểm đích là mở
    return maze

# Hàm chính
def main():
    pygame.init()

    grid_size = 25  # Kích thước grid
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    # Tạo một mê cung dễ (hoặc bạn có thể thay đổi các hàm khác)
    easy_maze = generate_easy_maze(grid_size)

    # Vẽ mê cung
    screen.fill((255, 255, 255))  # Màu nền trắng
    draw_maze(easy_maze, grid_size, screen)
    pygame.display.flip()

    # Chờ cho đến khi đóng cửa sổ
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

# Chạy hàm main
if __name__ == "__main__":
    main()
