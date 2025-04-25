import numpy as np
import matplotlib.pyplot as plt

class RRT3D:
    def __init__(self, n_iter=100, min_dist=2.0, search_space=[0, 100, 0, 100,0,100], goal=None):
        """
        Khởi tạo RRT
        :param n_iter: Số vòng lặp (số bước mở rộng cây)
        :param min_dist: Khoảng cách tối thiểu giữa các điểm (độ dài bước)
        :param search_space: Miền tìm kiếm [xmin, xmax, ymin, ymax]
        :param goal: Đích (nếu có), dạng tuple (x, y)
        """
        self.n_iter = n_iter
        self.min_dist = min_dist
        self.search_space = search_space
        self.goal = goal
         # Điểm bắt đầu: tâm không gian 3D
        start = ((search_space[0] + search_space[1]) / 2,
                 (search_space[2] + search_space[3]) / 2,
                 (search_space[4] + search_space[5]) / 2)
        self.tree = [start]      # Danh sách các nút đã khám phá
        self.edges = []          # Danh sách các cạnh nối các nút
        self.done = False

    def random_point(self):
        """ Sinh ra một điểm ngẫu nhiên trong không gian tìm kiếm """
        prob = np.random.rand()
        print(prob)
        if (prob <= 0.1):
            x = self.goal[0]
            y = self.goal[1]
            z = self.goal[2]
        else:
            x = np.random.uniform(self.search_space[0], self.search_space[1])
            y = np.random.uniform(self.search_space[2], self.search_space[3])
            z = np.random.uniform(self.search_space[4], self.search_space[5])
        return (x, y, z)

    def nearest_neighbor(self, point):
        """ Tìm điểm trong cây gần điểm 'point' nhất """
        tree_array = np.array(self.tree)
        # Tính khoảng cách Euclid cho các tọa độ
        distances = np.sqrt((tree_array[:, 0] - point[0])**2 +
                            (tree_array[:, 1] - point[1])**2 +
                            (tree_array[:, 2] - point[2])**2)
        idx = np.argmin(distances)
        return self.tree[idx]

    def steer(self, from_node, to_point):
        """
        Chọn điểm khám phá tiếp theo:
         - Nếu khoảng cách từ from_node đến to_point < min_dist thì trả về to_point
         - Ngược lại, trả về điểm cách from_node min_dist theo hướng từ from_node tới to_point
        """
        dx = to_point[0] - from_node[0]
        dy = to_point[1] - from_node[1]
        dz = to_point[2] - from_node[2]
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        if distance < self.min_dist:
            return to_point
        else:
            ratio = self.min_dist / distance
            new_x = from_node[0] + dx * ratio
            new_y = from_node[1] + dy * ratio
            new_z = from_node[2] + dz * ratio
            return (new_x, new_y, new_z)

    def add_point(self, from_node, new_node):
        """ Thêm điểm mới vào cây và ghi lại cạnh nối """
        
        self.tree.append(new_node)
        self.edges.append((from_node, new_node))
        if (new_node == self.goal):
            self.done = True

def main():
    # Thiết lập các tham số cho RRT3D
    n_iter = 200
    min_dist = 5.0
    # Không gian tìm kiếm dưới dạng [xmin, xmax, ymin, ymax, zmin, zmax]
    search_space = [0, 100, 0, 100, 0, 100]
    goal = (90, 90, 90)  # Ví dụ đích cần đạt, có thể bỏ qua nếu không có
    rrt = RRT3D(n_iter, min_dist, search_space, goal)

    # Khởi tạo đồ thị 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(search_space[0], search_space[1])
    ax.set_ylim(search_space[2], search_space[3])
    ax.set_zlim(search_space[4], search_space[5])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("3D RRT - Phát triển cây từng bước")
    plt.ion()  # Bật chế độ interactive

    # Vẽ điểm bắt đầu (ở trung tâm không gian)
    start = rrt.tree[0]
    ax.scatter(start[0], start[1], start[2], color='r', s=50)
    plt.draw()
    plt.pause(0.1)

    # Vòng lặp mở rộng cây
    for i in range(n_iter):
        if rrt.done == True:
            print(f'Done after {i} iterations')
            break
        # Sinh một điểm ngẫu nhiên trong không gian 3D
        rand_point = rrt.random_point()
        # Tìm điểm gần nhất trong cây
        nearest = rrt.nearest_neighbor(rand_point)
        # Tính điểm tiếp theo dựa trên hàm steer
        new_node = rrt.steer(nearest, rand_point)
        # Thêm điểm mới vào cây
        rrt.add_point(nearest, new_node)

        # Vẽ cạnh nối từ điểm nearest đến new_node
        ax.plot([nearest[0], new_node[0]],
                [nearest[1], new_node[1]],
                [nearest[2], new_node[2]], color='b')
        # Vẽ điểm new_node
        ax.scatter(new_node[0], new_node[1], new_node[2], color='g', s=10)
        plt.draw()
        plt.pause(0.05)

    # Vẽ đích nếu có
    if rrt.goal is not None:
        ax.scatter(rrt.goal[0], rrt.goal[1], rrt.goal[2], color='g', s=80)

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
