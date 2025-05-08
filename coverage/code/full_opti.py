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
scale     = 200.0   # Gauss spread
N         = 3       # số robot
alpha     = 0.3     # Lloyd step-size
max_iters = 100     # số bước tối đa
tol       = 1.0     # ngưỡng hội tụ (pixel)
down_samp = 5       # down-sampling cho Voronoi
# --------------------------------

Point = namedtuple('Point',['x','y'])

def point_in_poly(x, y, poly):
    inside = False
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[(i+1) % len(poly)]
        if ((yi>y) != (yj>y)) and (x < (xj-xi)*(y-yi)/(yj-yi) + xi):
            inside = not inside
    return inside

# tính bounding-box của polygon
min_x = min(p[0] for p in polygon_points)
max_x = max(p[0] for p in polygon_points)
min_y = min(p[1] for p in polygon_points)
max_y = max(p[1] for p in polygon_points)

# full mask & down-sampled mask
full_mask = [(x,y)
    for x in range(min_x, max_x+1)
    for y in range(min_y, max_y+1)
    if point_in_poly(x,y,polygon_points)]
mask_ds = full_mask[::down_samp]

def phi(x, y, cx, cy):
    dx = (x-cx)/scale
    dy = (y-cy)/scale
    return math.exp(-(dx*dx + dy*dy))

def gen_colors(n):
    cols = []
    for i in range(n):
        c = pygame.Color(0)
        c.hsva = (i*360/n, 100, 100, 100)
        cols.append((c.r, c.g, c.b))
    return cols

# Hàm chính
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Coverage Control with Voronoi")
    clock = pygame.time.Clock()

    state = 'waiting'      # 'waiting' chờ click, 'running' chạy Lloyd
    clicked_pt = None
    density_surf = None
    voronoi_surf = None
    converged = False
    robots = []
    trails = []
    colors = gen_colors(N)
    it = 0

    # lần đầu: nền trắng + polygon
    screen.fill((255,255,255))
    pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)
    pygame.display.flip()

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1 and state == 'waiting':
                clicked_pt = ev.pos
                state = 'running'
                # tạo density surface
                density_surf = pygame.Surface((WIDTH, HEIGHT))
                density_surf.fill((255,255,255))
                for x,y in full_mask:
                    v = phi(x, y, *clicked_pt)
                    c = int(v * 255)
                    density_surf.set_at((x,y), (c,c,c))
                # khởi robots ngẫu nhiên + trails
                robots = []
                while len(robots) < N:
                    x = random.randint(min_x, max_x)
                    y = random.randint(min_y, max_y)
                    if point_in_poly(x, y, polygon_points):
                        robots.append(Point(x,y))
                trails = [[(r.x, r.y)] for r in robots]
                it = 0

        # vẽ mỗi frame
        if density_surf:
            screen.blit(density_surf, (0,0))
        else:
            screen.fill((255,255,255))

        pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)

        # chạy Lloyd nếu chưa hội tụ và chưa quá số bước
        if state == 'running' and not converged and it < max_iters:
            # tính Voronoi rời rạc trên mask_ds
            regions = {i: [] for i in range(N)}
            for x,y in mask_ds:
                j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                regions[j].append((x,y))
            # tính centroid trọng số + cập nhật robots
            newr = []
            moves = []
            for i in range(N):
                pts = regions[i]
                if not pts:
                    newr.append(robots[i])
                    moves.append(0)
                else:
                    ws = xs = ys = 0.0
                    for x,y in pts:
                        w = phi(x, y, *clicked_pt)
                        ws += w; xs += w*x; ys += w*y
                    cx, cy = xs/ws, ys/ws
                    nx = robots[i].x + alpha*(cx - robots[i].x)
                    ny = robots[i].y + alpha*(cy - robots[i].y)
                    newr.append(Point(nx, ny))
                    moves.append(math.hypot(nx-robots[i].x, ny-robots[i].y))
                # lưu trail
                trails[i].append((newr[i].x, newr[i].y))
            # kiểm tra hội tụ
            max_move = max(moves)
            robots = newr
            it += 1
            if max_move < tol:
                converged = True
                # tạo voronoi surface
                voronoi_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                for x,y in full_mask:
                    j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                    rcol = colors[j]
                    voronoi_surf.set_at((x,y), (*rcol, 80))
                print(f"Hội tụ sau {it} bước, max_move={max_move:.4f}")

        # nếu đã hội tụ, vẽ Voronoi overlay
        if converged and voronoi_surf:
            screen.blit(voronoi_surf, (0,0))

        # vẽ trails và robots
        for i, trail in enumerate(trails):
            if len(trail) > 1:
                pygame.draw.lines(screen, colors[i], False, trail, 2)
            if robots:
                pygame.draw.circle(screen, colors[i], (int(robots[i].x), int(robots[i].y)), 6)

        # vẽ tâm click
        if clicked_pt:
            pygame.draw.circle(screen, (255,255,255), clicked_pt, 8)
            pygame.draw.circle(screen, (0,0,0), clicked_pt, 8, 2)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()