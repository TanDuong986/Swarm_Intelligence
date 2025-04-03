import random
import pygame
import json
import os
from heapq import heappush, heappop
import time

# Hàm tạo màu dựa trên cost
def get_cost_color(cost, max_cost):
    if cost == float('inf'):
        return (0, 0, 0)  # Đen cho vật cản
    # Chuyển cost thành gradient từ xanh (thấp) đến đỏ (cao)
    normalized = min(cost / max_cost, 1.0)
    r = int(255 * normalized)
    g = int(255 * (1 - normalized))
    b = 0
    return (r, g, b)

class Node:
    def __init__(self, x, y, is_obstacle=False, cost=1):
        self.x = x
        self.y = y
        self.is_obstacle = is_obstacle
        self.neighbors = []
        self.cost = float('inf') if is_obstacle else cost  # Cost mặc định
        self.g_score = float('inf')  # Dùng cho A*
        self.h_score = float('inf') # heuristic
        self.f_score = float('inf')  # Dùng cho A*
        self.parent = None

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

class Graph:
    def __init__(self, grid_size, use_random=False, obstacle_ratio=0.3, json_file=None):
        self.grid_size = grid_size
        self.nodes = {}
        self.start = None
        self.goal = None
        self.path_log = []
        self.max_cost = grid_size * 2

        # Khởi tạo từ file JSON hoặc ngẫu nhiên
        if json_file and os.path.exists(json_file):
            self.load_from_json(json_file)
        else:
            # Khởi tạo tất cả ô là đường đi với cost ngẫu nhiên
            for x in range(grid_size):
                for y in range(grid_size):
                    cost = 1  # Mặc định cost là 1
                    self.nodes[(x, y)] = Node(x, y, is_obstacle=False, cost=cost)

            # Đặt điểm start và goal
            self.set_start(0, 0)  # Start ở góc trên trái
            self.goal = self.get_random_free_cell()
            while self.goal == self.start:  # Đảm bảo goal khác start
                self.goal = self.get_random_free_cell()

        # Thiết lập neighbors
        for x in range(grid_size):
            for y in range(grid_size):
                current_node = self.nodes[(x, y)]
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < grid_size and 0 <= ny < grid_size: # nếu không kịch trần thì thêm class node vào list hàng xóm của nono
                        current_node.add_neighbor(self.nodes[(nx, ny)])

        if not self.goal: # nếu chưa có hàng xóm thì lấy một điểm ngẫu nhiên 
            self.goal = self.get_random_free_cell()

    def load_from_json(self, json_file): # đọc bản đồ từ file json thôi không có gìgì
        with open(json_file, 'r') as f:
            data = json.load(f)
            maze_data = data.get("data")  # Hoặc tên key tương ứng trong file của bạn

            # Tải dữ liệu từ ma trận
            for x in range(self.grid_size):
                for y in range(self.grid_size):
                    is_obstacle = (maze_data[x][y] == 1)
                    cost = float('inf') if is_obstacle else 1  # Vật cản có cost vô cùng
                    self.nodes[(x, y)] = Node(x, y, is_obstacle, cost)

    def get_node(self, x, y): # lấy instance node theo vị trí 
        return self.nodes.get((x, y))

    def get_random_free_cell(self):
        while True:
            x = random.randint(0, self.grid_size - 1)
            y = random.randint(0, self.grid_size - 1)
            if not self.nodes[(x, y)].is_obstacle:
                return self.nodes[(x, y)]

    def set_start(self, x, y):
        self.start = self.nodes[(x, y)]

    def set_goal(self, x, y):
        self.goal = self.nodes[(x, y)]

    def a_star(self):
        open_set = [(self.start.f_score, id(self.start), self.start)]  # Danh sách thay cho heap
        closed_set = set()
        self.start.g_score = 0
        self.start.f_score = abs(self.start.x - self.goal.x) + abs(self.start.y - self.goal.y)  # Khoảng cách Manhattan
        
        log = []  # Lưu log các bước (closed_set)
        frontier_log = []  # Lưu trạng thái open_set tại mỗi bước

        while open_set:
            # Lưu trạng thái hiện tại của open_set vào frontier_log
            frontier_log.append([(node.x, node.y) for _, _, node in open_set])

            # Tìm node có f_score nhỏ nhất trong open_set
            current_tuple = min(open_set, key=lambda x: (x[0], x[1]))
            current = current_tuple[2]
            open_set.remove(current_tuple)

            if current == self.goal:
                path = []
                total_explored = len(closed_set) + 1  # +1 để tính cả goal
                final_cost = current.g_score  # Chi phí thực tế đến goal
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1], log, frontier_log, total_explored, final_cost

            closed_set.add(current)
            log.append((current.x, current.y))

            for neighbor in current.neighbors:
                if neighbor in closed_set or neighbor.is_obstacle:
                    continue
                tentative_g_score = current.g_score + neighbor.cost
                
                # if abs(neighbor.x - current.x) ==1 and abs(neighbor.y - current.y) ==1:
                #     move_cost = 1.41
                # else:
                #     move_cost = 1
                # tentative_g_score = current.g_score + move_cost


                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current
                    neighbor.g_score = tentative_g_score
                    neighbor.f_score = neighbor.g_score + abs(neighbor.x - self.goal.x) + abs(neighbor.y - self.goal.y)
                    
                    neighbor_in_open = False
                    for item in open_set:
                        if item[2] == neighbor:
                            neighbor_in_open = True
                            open_set.remove(item)
                            open_set.append((neighbor.f_score, id(neighbor), neighbor))
                            break
                    if not neighbor_in_open:
                        open_set.append((neighbor.f_score, id(neighbor), neighbor))

        return [], log, frontier_log, len(closed_set), 0  # Không tìm thấy đường

    def bfs(self):
        from collections import deque  # Dùng deque làm queue
        
        open_set = deque([self.start])  # Queue cho BFS
        closed_set = set()
        self.start.g_score = 0  # Chi phí từ start đến node
        
        log = []  # Lưu log các bước (closed_set)
        frontier_log = []  # Lưu trạng thái open_set tại mỗi bước

        while open_set:
            # Lưu trạng thái hiện tại của open_set vào frontier_log
            frontier_log.append([(node.x, node.y) for node in open_set])
            
            current = open_set.popleft()  # Lấy node đầu tiên trong queue

            if current == self.goal:
                path = []
                total_explored = len(closed_set) + 1  # +1 để tính cả goal
                final_cost = current.g_score  # Chi phí thực tế đến goal
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1], log, frontier_log, total_explored, final_cost

            if current not in closed_set:
                closed_set.add(current)
                log.append((current.x, current.y))

                for neighbor in current.neighbors:
                    # Chỉ thêm neighbor nếu không phải vật cản và chưa được khám phá
                    if not neighbor.is_obstacle and neighbor not in closed_set and neighbor not in open_set:
                        neighbor.parent = current
                        neighbor.g_score = current.g_score + neighbor.cost
                        open_set.append(neighbor)

        return [], log, frontier_log, len(closed_set), 0  # Không tìm thấy đường
    
    def dfs(self):
        open_set = [self.start]  # Stack cho DFS
        closed_set = set()
        self.start.g_score = 0  # Chi phí từ start đến node
        
        log = []  # Lưu log các bước (closed_set)
        frontier_log = []  # Lưu trạng thái open_set tại mỗi bước

        while open_set:
            # Lưu trạng thái hiện tại của open_set vào frontier_log
            frontier_log.append([(node.x, node.y) for node in open_set])
            
            current = open_set.pop()  # Lấy node cuối cùng trong stack

            if current == self.goal:
                path = []
                total_explored = len(closed_set) + 1  # +1 để tính cả goal
                final_cost = current.g_score  # Chi phí thực tế đến goal
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1], log, frontier_log, total_explored, final_cost

            if current not in closed_set:
                closed_set.add(current)
                log.append((current.x, current.y))

                for neighbor in current.neighbors:
                    # Chỉ thêm neighbor nếu không phải vật cản và chưa được khám phá
                    if not neighbor.is_obstacle and neighbor not in closed_set and neighbor not in open_set:
                        neighbor.parent = current
                        neighbor.g_score = current.g_score + neighbor.cost
                        open_set.append(neighbor)

        return [], log, frontier_log, len(closed_set), 0  # Không tìm thấy đường

def draw_map(graph, grid_size, screen, timestep=-1, frontier_log=None):
    cell_size = 600 // grid_size
    screen.fill((255, 255, 255))  # Nền trắng

   

    # Vẽ grid với màu dựa trên trạng thái
    for x in range(grid_size):
        for y in range(grid_size):
            node = graph.get_node(x, y)

            if node == graph.start:
                color = (0, 0, 255)  # Màu xanh nước biển cho start
            elif node == graph.goal:
                color = (255, 0, 0)  # Màu đỏ cho goal
            elif (x, y) in graph.path_log[:timestep + 1]:  # Màu vàng cho điểm đã đi qua
                color = (255, 255, 0)
            elif node.is_obstacle:
                color = (0, 0, 0)  # Màu đen cho vật cản
            else:
                color = get_cost_color(node.cost, graph.max_cost)  # Màu gradient cho cost

            pygame.draw.rect(screen, color, (x * cell_size, y * cell_size, cell_size, cell_size))
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)


     # Vẽ các điểm biên (frontier) nếu có frontier_log
    if frontier_log and timestep >= 0 and timestep < len(frontier_log):
        for x, y in frontier_log[timestep]:
            pygame.draw.rect(screen, (212, 126, 252), (x * cell_size, y * cell_size, cell_size, cell_size))  # Màu xanh lá nhạt
            pygame.draw.rect(screen, (50, 50, 50), (x * cell_size, y * cell_size, cell_size, cell_size), 1)

    
    # Ve lai goal va duong vien
    pygame.draw.rect(screen, (255, 0, 0), (graph.goal.x * cell_size, graph.goal.y * cell_size, cell_size, cell_size)) # Ve them goal khi hoan thanh
    pygame.draw.rect(screen, (50, 50, 50), (graph.goal.x * cell_size, graph.goal.y * cell_size, cell_size, cell_size), 1)

    # Vẽ đường đi tối ưu khi hoàn thành
    if timestep >= len(graph.path_log) - 1:
        node = graph.goal
        while node:
            pygame.draw.circle(screen, (207, 85, 37), (node.x * cell_size + cell_size // 2, node.y * cell_size + cell_size // 2), cell_size // 4)
            node = node.parent
        save_image(screen, f"maze_final.png")

def save_image(screen, filename):
    pygame.image.save(screen, filename)

def main():
    pygame.init()
    grid_size = 25
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Maze Path Planning")

    # Tạo graph
    graph = Graph(grid_size, json_file="map/none.json")
    graph.set_start(5, 8) #(x, y theo hệ trục của ảnhh)
    graph.set_goal(1, 23)

    # Tìm đường bằng A*
    path, log, frontier_log, total_explored, final_cost = graph.bfs()
    graph.path_log = log  # Lưu log để vẽ từng bước

    # In thông tin
    if path:
        print(f"Tổng số điểm cần khám phá: {total_explored}")
        print(f"Chi phí thực tế đến đích: {final_cost}")
    else:
        print("Không tìm thấy đường đi!")

    clock = pygame.time.Clock()
    running = True
    timestep = 0
    start = time.time()
    is_paused = False
    search_done = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    is_paused = not is_paused

        if not search_done and not is_paused:
            if time.time() - start > 0.05:
                timestep = (timestep + 1) % (len(graph.path_log) + 1)
                start = time.time()
                if timestep == len(graph.path_log):
                    search_done = True

        # Vẽ bản đồ với frontier_log
        draw_map(graph, grid_size, screen, timestep, frontier_log)
        pygame.display.flip()
        clock.tick(24)

    pygame.quit()

if __name__ == "__main__":
    main()
