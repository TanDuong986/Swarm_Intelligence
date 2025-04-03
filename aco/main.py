import random
import pygame
import math
import sys
sys.setrecursionlimit(10000)  # Thay đổi giới hạn đệ quy (thử tăng lên nếu cần)

# Hàm tạo màu ngẫu nhiên
def random_color():
    return random.choice([[255, 0, 255], [255, 255, 255]])

class Node:
    def __init__(self, x, y, is_obstacle=False):
        self.x = x
        self.y = y
        self.is_obstacle = is_obstacle  # Xác định liệu ô có phải vật cản không
        self.neighbors = []  # Các node kề
        self.weight = 1 if not is_obstacle else float('inf')  # Trọng số, vật cản có trọng số vô cùng
        self.heuristic = 0  # Heuristic, dùng cho A*
    
    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def set_heuristic(self, target_x, target_y):
        # Tính toán heuristic sử dụng khoảng cách Manhattan
        self.heuristic = abs(self.x - target_x) + abs(self.y - target_y)
        
class Graph:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.nodes = {}
        
        # Khởi tạo các node
        for x in range(grid_size):
            for y in range(grid_size):
                self.nodes[(x, y)] = Node(x, y)
        
        # Thiết lập các neighbors (các ô liền kề)
        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                # Thêm các neighbor liền kề (trái, phải, trên, dưới)
                if x > 0:
                    current_node.add_neighbor(self.nodes[(x - 1, y)])  # Trái
                if x < grid_size - 1:
                    current_node.add_neighbor(self.nodes[(x + 1, y)])  # Phải
                if y > 0:
                    current_node.add_neighbor(self.nodes[(x, y - 1)])  # Trên
                if y < grid_size - 1:
                    current_node.add_neighbor(self.nodes[(x, y + 1)])  # Dưới

    def set_obstacle(self, x, y):
        self.nodes[(x, y)].is_obstacle = True
        self.nodes[(x, y)].weight = float('inf')

    def set_heuristic(self, target_x, target_y):
        for node in self.nodes.values():
            node.set_heuristic(target_x, target_y)

    def get_node(self, x, y):
        return self.nodes.get((x, y))

# Hàm tạo mê cung với thuật toán recursive backtracking
def generate_maze(grid_size, difficulty):
    graph = Graph(grid_size)
    # Khởi tạo mê cung với tất cả các vật cản
    for x in range(grid_size):
        for y in range(grid_size):
            if (x % 2 == 0 and y % 2 == 0):  # Tạo tường xung quanh các ô trống
                graph.set_obstacle(x, y)
    
    # Tạo mê cung bằng thuật toán backtracking
    def carve_maze(x, y):
        # Các hướng di chuyển: phải, trái, xuống, lên
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(directions)  # Xáo trộn hướng di chuyển để tạo sự ngẫu nhiên

        for dx, dy in directions:
            nx, ny = x + dx * 2, y + dy * 2  # Tiến xa hai ô để tạo lối đi

            # Kiểm tra nếu ô mới nằm trong phạm vi và chưa được xử lý
            if 0 < nx < grid_size and 0 < ny < grid_size and not graph.get_node(nx, ny).is_obstacle:
                # Xóa tường giữa ô hiện tại và ô mới
                graph.set_obstacle(x + dx, y + dy)
                carve_maze(nx, ny)  # Tiếp tục đệ quy để tạo lối đi mới
    
    # Chọn điểm bắt đầu
    start_x, start_y = 1, 1
    carve_maze(start_x, start_y)
    return graph

# Hàm vẽ grid map từ graph
def draw_map(graph, grid_size, screen):
    cell_size = 600 // grid_size
    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y)
            color = [255, 255, 255] if node.weight == 1 else [255, 0, 255]  # Trắng cho ô trống, hồng cho vật cản
            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (0, 0, 0), (x * cell_size, y * cell_size, cell_size, cell_size), 1)

# Hàm chính
def main():
    pygame.init()
    grid_size = 25  # Kích thước grid
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    # Tạo mê cung với độ khó dễ
    difficulty = 0.3  # 30% vật cản
    graph = generate_maze(grid_size, difficulty)

    # Vẽ mê cung
    screen.fill((255, 255, 255))  # Màu nền trắng
    draw_map(graph, grid_size, screen)
    pygame.display.flip()

    # Chờ cho đến khi đóng cửa sổ
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()

if __name__ == "__main__":
    main()
