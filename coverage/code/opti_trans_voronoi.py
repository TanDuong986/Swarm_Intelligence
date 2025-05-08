import pygame, sys, math, random
from collections import namedtuple

# -------- CẤU HÌNH CHÍNH --------
WIDTH, HEIGHT = 1200, 700
polygon_points = [
    (50, 650),
    (1100, 600),
    (1000, 50),
    (600, 20),
    (60, 60)
]
scale      = 200.0   # Gauss spread
N          = 3       # số robot
alpha      = 0.3     # Lloyd step-size
max_iters  = 100     # số bước tối đa
tol        = 1.0     # ngưỡng hội tụ (pixel)
down_samp  = 5       # down-sampling cho Voronoi
vor_alpha  = 20      # độ mờ cuối cùng của Voronoi overlay (0-255)
# --------------------------------

Point = namedtuple('Point',['x','y'])

def point_in_poly(x, y, poly):
    inside = False
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[(i+1) % len(poly)]
        if ((yi > y) != (yj > y)) and (x < (xj-xi)*(y-yi)/(yj-yi) + xi):
            inside = not inside
    return inside

# Tính bounding-box của polygon
min_x = min(p[0] for p in polygon_points)
max_x = max(p[0] for p in polygon_points)
min_y = min(p[1] for p in polygon_points)
max_y = max(p[1] for p in polygon_points)

# Full mask và down-sampled mask
full_mask = [
    (x, y)
    for x in range(min_x, max_x+1)
    for y in range(min_y, max_y+1)
    if point_in_poly(x, y, polygon_points)
]
mask_ds = full_mask[::down_samp]

def phi(x, y, cx, cy):
    dx = (x - cx) / scale
    dy = (y - cy) / scale
    return math.exp(-(dx*dx + dy*dy))

def gen_colors(n):
    cols = []
    for i in range(n):
        c = pygame.Color(0)
        c.hsva = (i * 360 / n, 100, 100, 100)
        cols.append((c.r, c.g, c.b))
    return cols

# Lloyd một bước, trả về (new_positions, moves)
def lloyd_step(robots, clicked_pt):
    regions = {i: [] for i in range(N)}
    for x, y in mask_ds:
        j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
        regions[j].append((x, y))
    newr, moves = [], []
    for i in range(N):
        pts = regions[i]
        if not pts:
            newr.append(robots[i]); moves.append(0); continue
        ws = xs = ys = 0.0
        for x, y in pts:
            w = phi(x, y, *clicked_pt)
            ws += w; xs += w*x; ys += w*y
        cx, cy = xs/ws, ys/ws
        nx = robots[i].x + alpha * (cx - robots[i].x)
        ny = robots[i].y + alpha * (cy - robots[i].y)
        newr.append(Point(nx, ny))
        moves.append(math.hypot(nx-robots[i].x, ny-robots[i].y))
    return newr, moves

# Hàm chính
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Coverage Control AutoRun + Alpha Example")
    clock = pygame.time.Clock()

    # Chờ click cho tâm phân phối
    clicked_pt = None
    while clicked_pt is None:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                clicked_pt = ev.pos
        clock.tick(30)

    # Tạo surface màu cho robots
    colors = gen_colors(N)  # Định nghĩa colors ở đây

    # Tạo density surface
    density_surf = pygame.Surface((WIDTH, HEIGHT))
    density_surf.fill((255,255,255))
    for x, y in full_mask:
        v = phi(x, y, *clicked_pt)
        c = int(v * 255)
        density_surf.set_at((x, y), (c, c, c))

    # Khởi robots + trails
    robots = []
    while len(robots) < N:
        x = random.randint(min_x, max_x)
        y = random.randint(min_y, max_y)
        if point_in_poly(x, y, polygon_points):
            robots.append(Point(x, y))
    trails = [[(r.x, r.y)] for r in robots]

    # Auto-run Lloyd
    converged = False
    it = 0
    while not converged and it < max_iters:
        robots, moves = lloyd_step(robots, clicked_pt)
        for i, r in enumerate(robots): trails[i].append((r.x, r.y))
        it += 1
        if max(moves) < tol:
            converged = True

    # Tạo Voronoi overlay với per-surface alpha
    voronoi_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    for x, y in full_mask:
        j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
        rcol = colors[j]
        voronoi_surf.set_at((x, y), (*rcol, 255))
    # Áp dụng alpha toàn cục
    voronoi_surf.set_alpha(vor_alpha)
    print(f"Hội tụ sau {it} bước, Voronoi alpha = {voronoi_surf.get_alpha()}")

    # Hiển thị kết quả cuối
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
        # vẽ density + polygon
        screen.blit(density_surf, (0, 0))
        pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)
        # vẽ overlay Voronoi
        screen.blit(voronoi_surf, (0, 0))
        # vẽ trails & robots
        for i, trail in enumerate(trails):
            if len(trail) > 1:
                pygame.draw.lines(screen, colors[i], False, trail, 2)
            pygame.draw.circle(screen, colors[i], (int(robots[i].x), int(robots[i].y)), 6)
        # điểm click
        pygame.draw.circle(screen, (255,255,255), clicked_pt, 8)
        pygame.draw.circle(screen, (0,0,0), clicked_pt, 8, 2)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()