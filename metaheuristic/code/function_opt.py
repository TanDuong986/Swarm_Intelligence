import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

#w = w_max - (w_max - w_min) * iteration / n_iterations # càng ra biên thì inertial càng nhỏ lại 

# --------- Định nghĩa hàm Rastrigin ---------
def rastrigin(X):
    x, y = X
    return 20 + x**2 + y**2 - 10 * (np.cos(2 * np.pi * x) + np.cos(2 * np.pi * y))

# --------- Tham số PSO và khởi tạo ---------
np.random.seed(42)
num_particles = 40
num_dimensions = 2
n_iterations = 60
w = 1.6       # inertia
c1 = 1.5      # cognitive
c2 = 1.5      # social

bounds = [-10, 10]
positions = np.random.uniform(bounds[0], bounds[1], (num_particles, num_dimensions))
velocities = np.random.uniform(-1, 1, (num_particles, num_dimensions))
pbest_positions = positions.copy()
pbest_scores = np.array([rastrigin(pos) for pos in positions])
gbest_idx = np.argmin(pbest_scores)
gbest_position = pbest_positions[gbest_idx].copy()
gbest_score = pbest_scores[gbest_idx]

# Lưu lịch sử vị trí cho animation
positions_history = []
gbest_history = []

# --------- Chạy PSO & lưu lịch sử ---------
for iteration in range(n_iterations):
    positions_history.append(positions.copy())
    gbest_history.append(gbest_position.copy())
    for i in range(num_particles):
        r1 = np.random.rand(num_dimensions)
        r2 = np.random.rand(num_dimensions)
        velocities[i] = (w * velocities[i]
                        + c1 * r1 * (pbest_positions[i] - positions[i])
                        + c2 * r2 * (gbest_position - positions[i]))
        positions[i] += velocities[i]
        positions[i] = np.clip(positions[i], bounds[0], bounds[1])
        score = rastrigin(positions[i])
        if score < pbest_scores[i]:
            pbest_scores[i] = score
            pbest_positions[i] = positions[i].copy()
            if score < gbest_score:
                gbest_score = score
                gbest_position = positions[i].copy()
positions_history.append(positions.copy())
gbest_history.append(gbest_position.copy())

# --------- Vẽ mặt 3D (contour) cho animation ---------
x = np.linspace(bounds[0], bounds[1], 400)
y = np.linspace(bounds[0], bounds[1], 400)
X, Y = np.meshgrid(x, y)
Z = rastrigin([X, Y])

fig, ax = plt.subplots(figsize=(8, 6))
levels = np.logspace(0, 2, 20)
contour = ax.contourf(X, Y, Z, levels=50, cmap='viridis', alpha=0.7)
plt.colorbar(contour, ax=ax)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('PSO Optimization of Rastrigin Function')

# Vẽ particle ban đầu & gbest
particle_scat = ax.scatter([], [], c='red', s=40, label='Particles', alpha=0.8, edgecolor='k')
gbest_scat = ax.scatter([], [], c='yellow', s=80, marker='*', label='Global Best', edgecolor='k')

ax.legend(loc='upper right')

def init():
    particle_scat.set_offsets(np.empty((0, 2)))
    gbest_scat.set_offsets(np.empty((0, 2)))
    return particle_scat, gbest_scat


def animate(frame):
    pos = positions_history[frame]
    gbest = gbest_history[frame]
    particle_scat.set_offsets(pos)
    gbest_scat.set_offsets([gbest])
    ax.set_title(f'PSO Optimization - Step {frame+1}/{n_iterations}')
    return particle_scat, gbest_scat

ani = animation.FuncAnimation(fig, animate, frames=len(positions_history),
                              interval=100, blit=False, init_func=init)

plt.show()
