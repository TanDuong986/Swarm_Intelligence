import numpy as np
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
        x_min = self.center[0] - self.half - robot_radius
        x_max = self.center[0] + self.half + robot_radius
        y_min = self.center[1] - self.half - robot_radius
        y_max = self.center[1] + self.half + robot_radius
        return x_min <= point[0] <= x_max and y_min <= point[1] <= y_max

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

    def in_collision(self, point):
        return any(obs.in_collision(point, self.robot_radius) for obs in self.obstacles)

    def in_environment(self, point):
        x, y = point
        return (self.robot_radius <= x <= self.width - self.robot_radius and
                self.robot_radius <= y <= self.height - self.robot_radius)

    def clip_point(self, point):
        x = np.clip(point[0], self.robot_radius, self.width - self.robot_radius)
        y = np.clip(point[1], self.robot_radius, self.height - self.robot_radius)
        return np.array([x, y])

    def clip_path(self, path):
        return np.array([self.clip_point(p) for p in path])

    def path_length(self, path):
        return np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))

    def count_violations(self, path):
        start_violation = np.linalg.norm(path[0] - self.start) > self.robot_radius
        goal_violation = np.linalg.norm(path[-1] - self.goal) > self.robot_radius
        env_violations = sum(not self.in_environment(pt) for pt in path)
        coll_violations = sum(self.in_collision(pt) for pt in path)
        total = int(start_violation) + int(goal_violation) + env_violations + coll_violations
        details = {
            'start_violation': start_violation,
            'goal_violation': goal_violation,
            'environment_violations': env_violations,
            'collision_violations': coll_violations
        }
        return total, details

# ==================== Spline Path ====================
class SplinePath:
    def __init__(self, environment, control_points, resolution=100):
        self.env = environment
        self.control_points = np.array(control_points)
        self.resolution = resolution

    @classmethod
    def from_flat_list(cls, environment, var_list, resolution=100, normalized=True):
        pts = np.array(var_list).reshape(-1, 2)
        if normalized:
            pts[:, 0] *= environment.width
            pts[:, 1] *= environment.height
        return cls(environment, pts, resolution)

    def get_path(self):
        pts = np.vstack((self.env.start, self.control_points, self.env.goal))
        t = np.linspace(0, 1, len(pts))
        spline = CubicSpline(t, pts, bc_type='clamped')
        t_dense = np.linspace(0, 1, self.resolution)
        raw = spline(t_dense)
        return self.env.clip_path(raw)

# ==================== Cost Function ====================
def path_planning_cost(solution, penalties=(2, 2, 1, 10)):
    path = solution.get_path()
    length = solution.env.path_length(path)
    _, vio = solution.env.count_violations(path)
    cost = length
    if vio['start_violation']:
        cost *= (1 + penalties[0])
    if vio['goal_violation']:
        cost *= (1 + penalties[1])
    cost *= (1 + vio['environment_violations'] * penalties[2])
    cost *= (1 + vio['collision_violations'] * penalties[3])
    return cost, {'path': path, 'length': length, 'violations': vio, 'solution': solution}

# ==================== Particle & PSO ====================
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
    def __init__(
        self,
        num_vars,
        var_min,
        var_max,
        cost_function,
        pop_size=50,
        c1=1.5,
        c2=1.5,
        inertia=0.8,
        wdamp=0.99,
        reset_interval=None
    ):
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
        self.global_best_cost = np.inf
        self.global_best_details = None

    def initialize_swarm(self):
        self.swarm = []
        for _ in range(self.pop_size):
            pos = np.random.uniform(self.var_min, self.var_max, self.num_vars)
            vel = np.zeros(self.num_vars)
            cost, details = self.cost_function(pos)
            particle = Particle(pos, vel, cost, details)
            self.swarm.append(particle)
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = pos.copy()
                self.global_best_details = details

    def update_particle(self, particle):
        r1 = np.random.rand(self.num_vars)
        r2 = np.random.rand(self.num_vars)
        particle.velocity = (
            self.inertia * particle.velocity
            + self.c1 * r1 * (particle.best_position - particle.position)
            + self.c2 * r2 * (self.global_best_pos - particle.position)
        )
        particle.position += particle.velocity
        particle.position = np.clip(particle.position, self.var_min, self.var_max)
        cost, details = self.cost_function(particle.position)
        particle.cost = cost
        particle.details = details
        if cost < particle.best_cost:
            particle.best_cost = cost
            particle.best_position = particle.position.copy()
            particle.best_details = details
            if cost < self.global_best_cost:
                self.global_best_cost = cost
                self.global_best_pos = particle.position.copy()
                self.global_best_details = details

    def run(self, max_iter=100, callback=None):
        self.initialize_swarm()
        for iteration in range(1, max_iter + 1):
            if self.reset_interval and iteration % self.reset_interval == 0:
                self.initialize_swarm()
            for p in self.swarm:
                self.update_particle(p)
            self.inertia *= self.wdamp
            if callback is not None:
                callback(iteration, self.global_best_details)
            print(f"Iteration {iteration}: Best Cost = {self.global_best_cost:.4f}")
        return self.global_best_pos, self.global_best_cost, self.global_best_details

# ==================== Visualization ====================
class Visualizer:
    def __init__(self, environment):
        self.env = environment
        self.line = None
        self.scatter = None

    def __call__(self, iteration, best_details):
        sol = best_details['solution']
        path = best_details['path']
        cps = sol.control_points
        if self.line is None:
            plt.figure(figsize=(6,6))
            plot_environment(self.env)
            # plot path line
            self.line, = plt.plot(path[:,0], path[:,1], 'b-')
            # plot control points
            self.scatter = plt.scatter(cps[:,0], cps[:,1], marker='x', c='purple', s=80)
            plt.title(f"Iter {iteration}")
            plt.pause(0.1)
        else:
            # update path
            self.line.set_data(path[:,0], path[:,1])
            # update control points
            self.scatter.set_offsets(cps)
            plt.title(f"Iter {iteration}")
            plt.pause(0.1)

# ==================== Plot Helpers ====================
def plot_environment(env):
    ax = plt.gca()
    ax.set_aspect('equal')
    for obs in env.obstacles:
        if isinstance(obs, CircularObstacle):
            ax.add_patch(plt.Circle(obs.center, obs.radius, color='gray', alpha=0.5))
        else:
            ax.add_patch(plt.Rectangle(obs.center - obs.half, 2*obs.half, 2*obs.half, color='brown', alpha=0.5))
    ax.plot(env.start[0], env.start[1], 'go')
    ax.plot(env.goal[0], env.goal[1], 'ro')
    ax.set_xlim(0, env.width)
    ax.set_ylim(0, env.height)

# ==================== Main ====================
if __name__ == '__main__':
    env = Environment(width=100, height=100, robot_radius=1, start=[5,5], goal=[95,95])
    for center, r in [([30,30],10), ([70,40],15)]:
        env.add_obstacle(CircularObstacle(center, r))
    for center, s in [([40,70],25), ([60,20],8)]:
        env.add_obstacle(SquareObstacle(center, s))

    num_cp = 3
    resolution = 100
    cost_function = lambda var_list: path_planning_cost(
        SplinePath.from_flat_list(env, var_list, resolution, normalized=True)
    )

    solver = PSO(
        num_vars=2 * num_cp,
        var_min=0,
        var_max=1,
        cost_function=cost_function,
        pop_size=80,
        c1=2.0,
        c2=2.0,
        inertia=1.0,
        wdamp=0.99,
        reset_interval=20
    )

    viz = Visualizer(env)
    best_pos, best_cost, best_details = solver.run(max_iter=150, callback=viz)
    plt.show()
