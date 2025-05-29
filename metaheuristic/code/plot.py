import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import pandas as pd
import time

# ==================== PSO and Path Planning Core ====================
class Particle:
    def __init__(self, position, velocity, cost, details):
        self.position = position
        self.velocity = velocity
        self.cost = cost
        self.details = details
        self.best_position = position.copy()
        self.best_cost = cost
        self.best_details = details

class PSO:
    def __init__(self, num_vars, var_min, var_max, cost_function,
                 pop_size=50, c1=1.5, c2=1.5, inertia=0.8, wdamp=0.99, reset_interval=None):
        self.num_vars = num_vars
        self.var_min = var_min
        self.var_max = var_max
        self.cost_function = cost_function
        self.pop_size = pop_size
        self.c1 = c1
        self.c2 = c2
        self.inertia = inertia
        self.wdamp = wdamp
        self.reset_interval = reset_interval
        self.swarm = []
        self.global_best_pos = None
        self.global_best_cost = float('inf')
        self.global_best_details = None

    def initialize_swarm(self):
        self.swarm = []
        for _ in range(self.pop_size):
            pos = np.random.uniform(self.var_min, self.var_max, self.num_vars)
            vel = np.zeros(self.num_vars)
            cost, details = self.cost_function(pos)
            p = Particle(pos, vel, cost, details)
            self.swarm.append(p)
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = pos.copy()
                self.global_best_details = details

    def update_particle(self, p):
        r1 = np.random.rand(self.num_vars)
        r2 = np.random.rand(self.num_vars)
        p.velocity = (
            self.inertia * p.velocity
            + self.c1 * r1 * (p.best_position - p.position)
            + self.c2 * r2 * (self.global_best_pos - p.position)
        )
        p.position += p.velocity
        p.position = np.clip(p.position, self.var_min, self.var_max)
        cost, details = self.cost_function(p.position)
        p.cost = cost
        p.details = details
        if cost < p.best_cost:
            p.best_cost = cost
            p.best_position = p.position.copy()
            p.best_details = details
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = p.position.copy()
                self.global_best_details = details

    def run(self, max_iter=100, no_improve_thresh=20):
        self.initialize_swarm()
        best_prev = float('inf')
        no_improve = 0
        iters_conv = 0
        for i in range(1, max_iter+1):
            for p in self.swarm:
                self.update_particle(p)
            self.inertia *= self.wdamp
            if  best_prev - self.global_best_cost > 1e-3:
                best_prev = self.global_best_cost
                no_improve = 0
                iters_conv = i
            else:
                no_improve += 1
            if no_improve >= no_improve_thresh:
                break

            path = self.global_best_details['path']
            length = self.global_best_details['solution'].env.path_length(path)

            # print iteration info
            if i % 50 == 0 or i == max_iter:
                print(f"Iteration {i}: cost={self.global_best_cost:.4f}, length={length:.4f}, no_improve={no_improve}")

        path = self.global_best_details['path']
        return self.global_best_cost, path, iters_conv

# ==================== Obstacles ====================
class CircularObstacle:
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius
    def in_collision(self, pt, robot_radius=0):
        return np.linalg.norm(pt - self.center) <= self.radius + robot_radius

class SquareObstacle:
    def __init__(self, center, side):
        self.center = np.array(center)
        self.half = side/2
    def in_collision(self, pt, robot_radius=0):
        x, y = pt
        return (self.center[0]-self.half-robot_radius <= x <= self.center[0]+self.half+robot_radius and
                self.center[1]-self.half-robot_radius <= y <= self.center[1]+self.half+robot_radius)

# ==================== Environment ====================
class Environment:
    def __init__(self, width, height, robot_radius, start, goal):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = []
    def add_obstacle(self, o): self.obstacles.append(o)
    def in_collision(self, pt): return any(o.in_collision(pt, self.robot_radius) for o in self.obstacles)
    def clip_path(self, path):
        return np.clip(path, [self.robot_radius,self.robot_radius], [self.width-self.robot_radius,self.height-self.robot_radius])
    def path_length(self, path): return np.sum(np.linalg.norm(np.diff(path,axis=0),axis=1))
    def count_violations(self, path):
        start_v = np.linalg.norm(path[0]-self.start)>self.robot_radius
        goal_v  = np.linalg.norm(path[-1]-self.goal)>self.robot_radius
        env_v   = sum((pt[0]<self.robot_radius or pt[0]>self.width-self.robot_radius or pt[1]<self.robot_radius or pt[1]>self.height-self.robot_radius) for pt in path)
        coll_v  = sum(self.in_collision(pt) for pt in path)
        total = int(start_v)+int(goal_v)+env_v+coll_v
        details = {'start_violation':start_v,'goal_violation':goal_v,'environment_violations':env_v,'collision_violations':coll_v}
        return total, details

# ==================== Spline Path ====================
class SplinePath:
    def __init__(self, env, cps, resolution):
        self.env = env
        self.cps = np.array(cps)
        self.resolution = resolution
    @classmethod
    def from_vector(cls, env, vec, resolution):
        pts = np.array(vec).reshape(-1,2)
        pts[:,0] *= env.width
        pts[:,1] *= env.height
        return cls(env, pts, resolution)
    def get_path(self):
        pts = np.vstack((self.env.start, self.cps, self.env.goal))
        t = np.linspace(0,1,len(pts))
        cs = CubicSpline(t, pts, bc_type='clamped')
        dense = np.linspace(0,1,self.resolution)
        path = cs(dense)
        return self.env.clip_path(path)

# ==================== Parameter Sweep Helpers ====================
def run_pso_metrics(env, num_cp, resolution, pop_size, c1, c2, inertia, wdamp,
                    max_iter=400, no_improve_thresh=10):
    def cost_fn(vec):
        sol = SplinePath.from_vector(env, vec, resolution)
        path = sol.get_path()
        length = env.path_length(path)
        _, vio = env.count_violations(path)
        w_len, w_col, w_env = 10, 50, 5
        cost = w_len*length + w_col*vio['collision_violations'] + w_env*vio['environment_violations']
        return cost, {'solution': sol, 'path': path}

    solver = PSO(2*num_cp, 0, 1, cost_fn,
                 pop_size=pop_size, c1=c1, c2=c2,
                 inertia=inertia, wdamp=wdamp)
    best_cost, path, iters = solver.run(max_iter, no_improve_thresh)
    length = env.path_length(path)
    return best_cost, length, iters


def parameter_sweep(env, baseline, param_name, values):
    rows = []
    for v in values:
        print(f"Evaluating {param_name} = {v}")
        cfg = baseline.copy()
        cfg[param_name] = v
        
        start_t = time.time()
        cost, length, iters = run_pso_metrics(
            env,
            num_cp=cfg['num_cp'], resolution=cfg['resolution'],
            pop_size=cfg['pop_size'], c1=cfg['c1'], c2=cfg['c2'],
            inertia=cfg['inertia'], wdamp=cfg['wdamp'],max_iter=500, no_improve_thresh=10
        )
        elapsed = time.time() - start_t
        
        print(f"  -> cost={cost:.2f}, length={length:.2f}, iters={iters}, time={elapsed:.2f}s")
        rows.append({
            'parameter':      param_name,
            'value':          v,
            'best_cost':      cost,
            'path_length':    length,
            'iters_to_conv':  iters,
            'time_s':         elapsed     
        })
    return pd.DataFrame(rows)

# ==================== Main ====================
def main():
    # Chỉ khảo sát tham số, không dùng pygame
    width, height = 100, 100
    env = Environment(width, height, robot_radius=1, start=[5,5], goal=[95,95])
    # Thêm obstacles cố định
    for s in [([30,30],35),([70,30],35),([30,70],35),([70,70],35),([50,50],15)]:
        env.add_obstacle(SquareObstacle(*s))

    # Thiết lập tham số baseline
    baseline = {
        'pop_size':   60,
        'num_cp':     3,
        'c1':         2.5,
        'c2':         1.0,
        'inertia':    0.9,
        'wdamp':      0.85,
        'resolution': 150
    }
    # Dải giá trị khảo sát
    param_grids = {
        'pop_size':   [50,80,90,100,120,150],
        'num_cp':     [2,3,4,5,6,7],
        'c1':         [1.0,1.5,2.0,2.5,3.0,3.5],
        'c2':         [0.5,1.0,1.5,2.0,2.5,3.0],
        'inertia':    [0.5,0.65,0.8,0.95,1.1,1.5],
        'resolution': [50,80,100,130,150,200]
    }

    all_dfs = []
    # Khảo sát từng tham số
    for pname, pvals in param_grids.items():
        df = parameter_sweep(env, baseline, pname, pvals)
        # Lưu ảnh biểu đồ cho mỗi metric
        for metric in ['best_cost','path_length','iters_to_conv', 'time_s']:
            fig, ax = plt.subplots(figsize=(6,4))
            ax.plot(df['value'], df[metric], marker='o')
            ax.set_xlabel(pname)
            ax.set_ylabel(metric.replace('_',' ').title())
            ax.set_title(f"{metric.replace('_',' ').title()} vs {pname}")
            fig.tight_layout()
            fname = fr"metaheuristic\figure\{pname}_{metric}.png"
            fig.savefig(fname,dpi = 400)
            plt.close(fig)
            print(f"Saved plot: {fname}")
        all_dfs.append(df)

    # Ghi kết quả sweep ra CSV
    results = pd.concat(all_dfs, ignore_index=True)
    results.to_csv('pso_sweep_results.csv', index=False)
    print("Parameter sweep completed. Results saved to pso_sweep_results.csv")

if __name__ == '__main__':
    main()
