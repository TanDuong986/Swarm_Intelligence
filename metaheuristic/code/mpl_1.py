import numpy as np
from copy import deepcopy
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# ==================== Obstacles ====================
class CircularObstacle:
    def __init__(self, center, radius):
        self.center = np.array(center)
        self.radius = radius

    def in_collision(self, point, robot_radius=0):
        return np.linalg.norm(point - self.center) <= self.radius + robot_radius

class SquareObstacle:
    def __init__(self, center, side_length):
        self.center = np.array(center)
        self.half = side_length / 2

    def in_collision(self, point, robot_radius=0):
        # Expand square by robot radius
        x_min = self.center[0] - self.half - robot_radius
        x_max = self.center[0] + self.half + robot_radius
        y_min = self.center[1] - self.half - robot_radius
        y_max = self.center[1] + self.half + robot_radius
        return (x_min <= point[0] <= x_max) and (y_min <= point[1] <= y_max)

# ==================== Environment ====================
class Environment:
    def __init__(self, width=100, height=100, robot_radius=0.5, start=None, goal=None):
        self.width = width
        self.height = height
        self.robot_radius = robot_radius
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = []

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def clear_obstacles(self):
        self.obstacles = []

    def in_collision(self, point):
        for obs in self.obstacles:
            if obs.in_collision(point, self.robot_radius):
                return True
        return False

    def in_environment(self, point):
        return (self.robot_radius <= point[0] <= self.width - self.robot_radius) and \
               (self.robot_radius <= point[1] <= self.height - self.robot_radius)

    def clip_point(self, point):
        x = np.clip(point[0], self.robot_radius, self.width - self.robot_radius)
        y = np.clip(point[1], self.robot_radius, self.height - self.robot_radius)
        return np.array([x, y])

    def clip_path(self, path):
        return np.array([self.clip_point(p) for p in path])

    def path_length(self, path):
        return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

    def count_violations(self, path):
        details = {
            'start_violation': False,
            'goal_violation': False,
            'environment_violation_count': 0,
            'collision_violation_count': 0,
        }
        if np.linalg.norm(path[0] - self.start) > self.robot_radius:
            details['start_violation'] = True
        if np.linalg.norm(path[-1] - self.goal) > self.robot_radius:
            details['goal_violation'] = True

        for pt in path:
            if not self.in_environment(pt):
                details['environment_violation_count'] += 1
            if self.in_collision(pt):
                details['collision_violation_count'] += 1

        total_violations = (details['start_violation'] + details['goal_violation'] +
                             details['environment_violation_count'] + details['collision_violation_count'])
        return total_violations, details

# ==================== Spline Path ====================
class SplinePath:
    def __init__(self, environment, control_points, resolution=100):
        self.environment = environment
        self.control_points = np.array(control_points)
        self.resolution = resolution

    @staticmethod
    def from_flat_list(env, var_list, resolution=100, normalized=True):
        pts = np.array(var_list).reshape(-1, 2)
        if normalized:
            pts[:,0] *= env.width
            pts[:,1] *= env.height
        return SplinePath(env, pts, resolution)

    def get_path(self):
        pts = np.vstack((self.environment.start, self.control_points, self.environment.goal))
        t = np.linspace(0, 1, len(pts))
        cs = CubicSpline(t, pts, bc_type='clamped')
        tt = np.linspace(0, 1, self.resolution)
        path = cs(tt)
        return self.environment.clip_path(path)

# ==================== Cost Function ====================
def PathPlanningCost(sol, penalties=(2,2,1,50)):
    length = sol.environment.path_length(sol.get_path())
    _, d = sol.environment.count_violations(sol.get_path())
    cost = length * 10 
    if d['start_violation']:
        cost *= (1 + penalties[0])
    if d['goal_violation']:
        cost *= (1 + penalties[1])
    cost *= (1 + d['environment_violation_count'] * penalties[2])
    cost *= (1 + d['collision_violation_count'] * penalties[3])
    d.update({'path': sol.get_path(), 'length': length, 'cost': cost, 'sol': sol})
    return cost, d

def EnvCostFunction(env, num_cp, resolution):
    def fn(x):
        sol = SplinePath.from_flat_list(env, x, resolution, normalized=True)
        return PathPlanningCost(sol)
    return fn

# ==================== PSO ====================

def PSO(problem, max_iter=100, pop_size=50, c1=1.5, c2=1.5, w=0.8, wdamp=0.99, reset_interval=None, callback=None):
    num_var = problem['num_var']
    var_min, var_max = problem['var_min'], problem['var_max']
    cost_fn = problem['cost_function']

    # Particle template
    template = {'position': None, 'velocity': None, 'cost': None, 'details': None,
                'best': {'position': None, 'cost': np.inf, 'details': None}}

    # Initialize
    pop = []
    gbest = {'position': None, 'cost': np.inf, 'details': None}
    for _ in range(pop_size):
        p = deepcopy(template)
        p['position'] = np.random.uniform(var_min, var_max, num_var)
        p['velocity'] = np.zeros(num_var)
        p['cost'], p['details'] = cost_fn(p['position'])
        p['best'] = deepcopy(p)
        pop.append(p)
        if p['cost'] < gbest['cost']:
            gbest = deepcopy(p)

    # Main loop
    for it in range(1, max_iter+1):
        reset = reset_interval and (it % reset_interval == 0)
        if reset:
            for p in pop:
                p['position'] = np.random.uniform(var_min, var_max, num_var)
                p['velocity'] = np.zeros(num_var)

        for p in pop:
            if not reset:
                r1, r2 = np.random.rand(num_var), np.random.rand(num_var)
                p['velocity'] = (w*p['velocity'] + c1*r1*(p['best']['position'] - p['position']) \
                                 + c2*r2*(gbest['position'] - p['position']))
                p['position'] += p['velocity']
                p['position'] = np.clip(p['position'], var_min, var_max)

            p['cost'], p['details'] = cost_fn(p['position'])
            if p['cost'] < p['best']['cost']:
                p['best'] = deepcopy(p)
                if p['best']['cost'] < gbest['cost']:
                    gbest = deepcopy(p['best'])

        w *= wdamp
        print(f"Iteration {it}: Best Cost = {gbest['cost']:.4f}")
        if callback:
            callback({'it': it, 'gbest': gbest,
                      'pop': pop})
    return gbest, pop

# ==================== Plotting ====================
def plot_environment(env, ax=None):
    if ax is None:
        ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    # Plot circular
    for obs in env.obstacles:
        if isinstance(obs, CircularObstacle):
            circle = plt.Circle(obs.center, obs.radius, color='gray', alpha=0.5)
            ax.add_patch(circle)
        else:
            # square
            square = plt.Rectangle(obs.center - obs.half, 2*obs.half, 2*obs.half, color='brown', alpha=0.5)
            ax.add_patch(square)
    ax.plot(env.start[0], env.start[1], 'go', markersize=8)
    ax.plot(env.goal[0], env.goal[1], 'ro', markersize=8)

    

    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)


def plot_path(sol, ax=None, **kwargs):
    if ax is None:
        ax = plt.gca()
    path = sol.get_path()
    return ax.plot(path[:,0], path[:,1], **kwargs)

# ==================== Main ====================
def main():
    # Environment setup
    env = Environment(width=100, height=100, robot_radius=1, start=[5,5], goal=[95,95])
    # Circular obstacles
    circles = [([30,30],10), ([70,40],15)]
    for c in circles:
        env.add_obstacle(CircularObstacle(*c))
    # Square obstacles
    squares = [([40,70],25), ([60,20],8)]
    for s in squares:
        env.add_obstacle(SquareObstacle(*s))

    # PSO parameters
    num_cp = 3
    resolution = 100
    cost_fn = EnvCostFunction(env, num_cp, resolution)
    problem = {'num_var': 2*num_cp, 'var_min':0, 'var_max':1, 'cost_function': cost_fn}

    # Visualization callback
    line = None
    def cb(data):
        nonlocal line
        it = data['it']
        sol = data['gbest']['details']['sol']
        if it == 1:
            plt.figure(figsize=(6,6))
            plot_environment(env)
            line, = plot_path(sol, color='blue')
            plt.title(f"Iter {it}")
            plt.pause(0.05)
        else:
            x, y = sol.get_path().T
            line.set_data(x, y)
            plt.title(f"Iter {it}")
            plt.pause(0.05)

    # Run PSO
    best, pop = PSO(problem, max_iter=150, pop_size=80,
                     c1=2.0, c2=2.0, w=1.0, wdamp=0.99,
                     reset_interval=20, callback=cb)
    plt.show()

if __name__ == '__main__':
    main()
