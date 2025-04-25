import numpy as np
import matplotlib.pyplot as plt

class RRTStar:
    def __init__(self, start, goal, search_space, n_iter=500, step_size=5.0, radius=10.0):
        """
        Khởi tạo thuật toán RRT*.
        :param start: Điểm bắt đầu (x, y).
        :param goal: Điểm đích (x, y).
        :param search_space: Không gian tìm kiếm dưới dạng [xmin, xmax, ymin, ymax].
        :param n_iter: Số vòng lặp tối đa.
        :param step_size: Khoảng cách mở rộng mỗi bước.
        :param radius: Bán kính tìm kiếm các nút lân cận để thực hiện bước “rewiring”.
        """
        self.start = start
        self.goal = goal
        self.search_space = search_space
        self.n_iter = n_iter
        self.step_size = step_size
        self.radius = radius
        
        # Danh sách các nút hiện có trong cây.
        self.nodes = [start]
        # Dictionary chứa thông tin cha của mỗi nút.
        self.parent = {start: None}
        # Dictionary lưu chi phí từ điểm bắt đầu đến mỗi nút.
        self.cost = {start: 0}

    def sample_point(self):
        """ Sinh ra điểm ngẫu nhiên trong không gian tìm kiếm. """
        pb = np.random.rand()
        if pb < 0.2 :
            x_rand = self.goal[0]
            y_rand = self.goal[1]
        else:
            x_rand = np.random.uniform(self.search_space[0], self.search_space[1])
            y_rand = np.random.uniform(self.search_space[2], self.search_space[3])
        return (x_rand, y_rand)
    
    def nearest(self, x_rand):
        """ Tìm nút gần với điểm x_rand nhất trong cây. """
        nodes_array = np.array(self.nodes)
        diff = nodes_array - np.array(x_rand)
        dists = np.linalg.norm(diff, axis=1)
        idx = np.argmin(dists)
        return self.nodes[idx]
    
    def steer(self, x_nearest, x_rand):
        """
        Tạo nút mới theo hướng từ x_nearest tới x_rand, với bước tối đa là step_size.
        Nếu khoảng cách nhỏ hơn step_size thì lấy luôn x_rand.
        """
        vec = np.array(x_rand) - np.array(x_nearest)
        dist = np.linalg.norm(vec)
        if dist < self.step_size:
            return x_rand
        else:
            vec = vec / dist * self.step_size
            new_point = (x_nearest[0] + vec[0], x_nearest[1] + vec[1])
            return new_point
    
    def near(self, x_new):
        """
        Tìm các nút trong cây có khoảng cách đến x_new < radius.
        Đây là tập hợp các nút lân cận dùng để lựa chọn cha tối ưu và thực hiện rewire.
        """
        near_nodes = []
        for node in self.nodes:
            if np.linalg.norm(np.array(node) - np.array(x_new)) < self.radius:
                near_nodes.append(node)
        return near_nodes
    
    def collision_free(self, x1, x2):
        """
        Kiểm tra xem đoạn nối từ x1 đến x2 có va chạm hay không.
        Trong ví dụ này, giả sử không có vật cản nào (có thể bổ sung bằng cách kiểm tra occupancy grid hay các yếu tố hình học).
        """
        return True
    
    def retrieve_path(self):
        """
        Truy xuất chuỗi đường đi từ start đến goal dựa theo thông tin cha của mỗi nút.
        Trả về danh sách các điểm theo thứ tự từ điểm bắt đầu tới đích.
        """
        path = []
        node = self.goal
        while node is not None:
            path.append(node)
            node = self.parent.get(node)
        path.reverse()
        return path

    def run(self, animate=True):
        """
        Chạy thuật toán RRT* với số vòng lặp nhất định.
        Nếu animate=True thì trực quan hoá quá trình mở rộng cây.
        Trả về đường đi từ start đến goal nếu tìm được.
        """
        path = None
        fig, ax = None, None
        if animate:
            plt.ion()
            fig, ax = plt.subplots()
            ax.set_xlim(self.search_space[0], self.search_space[1])
            ax.set_ylim(self.search_space[2], self.search_space[3])
            # Vẽ điểm bắt đầu và đích
            ax.plot(self.start[0], self.start[1], "ro", markersize=5)
            ax.plot(self.goal[0], self.goal[1], "go", markersize=5)
        
        for i in range(self.n_iter):
            # Bước 1: Lấy điểm mẫu ngẫu nhiên
            x_rand = self.sample_point()
            # Bước 2: Tìm nút gần nhất trong cây
            x_nearest = self.nearest(x_rand)
            # Bước 3: Tạo nút mới theo hướng từ x_nearest tới x_rand
            x_new = self.steer(x_nearest, x_rand)
            if not self.collision_free(x_nearest, x_new):
                continue

            # Bước 4: Tìm các nút lân cận trong bán kính radius
            X_near = self.near(x_new)
            
            # Khởi tạo chi phí từ start đến x_new qua x_nearest
            c_min = self.cost[x_nearest] + np.linalg.norm(np.array(x_nearest) - np.array(x_new))
            x_min = x_nearest
            
            # Chọn cha tối ưu cho x_new từ tập các nút lân cận
            for x_near in X_near:
                if self.collision_free(x_near, x_new):
                    c_temp = self.cost[x_near] + np.linalg.norm(np.array(x_near) - np.array(x_new))
                    if c_temp < c_min:
                        x_min = x_near
                        c_min = c_temp

            # Thêm x_new vào cây với cha là x_min
            self.nodes.append(x_new)
            self.parent[x_new] = x_min
            self.cost[x_new] = c_min
            
            # Rewire: với mỗi x_near trong X_near, nếu nối từ x_new có chi phí thấp hơn thì cập nhật lại cha và chi phí.
            for x_near in X_near:
                if self.collision_free(x_new, x_near):
                    if self.cost[x_new] + np.linalg.norm(np.array(x_new) - np.array(x_near)) < self.cost[x_near]:
                        self.parent[x_near] = x_new
                        self.cost[x_near] = self.cost[x_new] + np.linalg.norm(np.array(x_new) - np.array(x_near))
            
            if animate:
                # Vẽ cạnh nối giữa x_new và cha của nó
                parent = self.parent[x_new]
                ax.plot([parent[0], x_new[0]], [parent[1], x_new[1]], "b-")
                ax.plot(x_new[0], x_new[1], "ro", markersize=3)
                plt.pause(0.01)
            
            # Kiểm tra nếu x_new đủ gần goal (ví dụ trong khoảng bước di chuyển)
            if np.linalg.norm(np.array(x_new) - np.array(self.goal)) < self.step_size:
                # Gán cha của đích là x_new và truy xuất path
                self.parent[self.goal] = x_new
                self.cost[self.goal] = self.cost[x_new] + np.linalg.norm(np.array(x_new)-np.array(self.goal))
                # Vẽ cạnh nối giữa x_new và goal
                if animate:
                    ax.plot([x_new[0], self.goal[0]], [x_new[1], self.goal[1]], "b-", linewidth=2)
                path = self.retrieve_path()
                print("Found path in iteration", i)
                break
        
        if animate:
            plt.ioff()
            plt.show()
        return path

def main():
    # Các tham số
    start = (10, 10)
    goal = (90, 90)
    search_space = [0, 100, 0, 100]
    
    # Khởi tạo RRT* với số vòng lặp tối đa là 500, bước di chuyển 2.0 và bán kính lân cận 10.0
    rrt_star = RRTStar(start, goal, search_space, n_iter=1000, step_size=2.0, radius=10.0)
    path = rrt_star.run(animate=True)
    
    if path is not None:
        print("Found path:")
        for point in path:
            print(point)
    else:
        print("No path found.")

if __name__ == '__main__':
    main()
