import pygame, sys, math, random
from collections import namedtuple
import csv
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
view_radius = 200   # bán kính cảm biến (pixel)
# --------------------------------

Point = namedtuple('Point',['x','y'])

# Kiểm tra điểm nằm trong đa giác
def point_in_poly(x, y, poly):
    inside = False
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[(i+1) % len(poly)]
        if ((yi>y) != (yj>y)) and (x < (xj-xi)*(y-yi)/(yj-yi) + xi):
            inside = not inside
    return inside

# Tạo mask full và down-sampled
min_x = min(p[0] for p in polygon_points)
max_x = max(p[0] for p in polygon_points)
min_y = min(p[1] for p in polygon_points)
max_y = max(p[1] for p in polygon_points)
full_mask = [(x,y) for x in range(min_x, max_x+1)
                    for y in range(min_y, max_y+1)
                    if point_in_poly(x,y,polygon_points)]
mask_ds  = full_mask[::down_samp]

# Mật độ thông tin Gaussian centered at clicked_pt
def phi(x, y, cx, cy):
    dx = (x-cx)/scale
    dy = (y-cy)/scale
    return math.exp(-(dx*dx + dy*dy))

# Sinh màu cho mỗi robot
def gen_colors(n):
    cols = []
    for i in range(n):
        c = pygame.Color(0)
        c.hsva = (i*360/n, 100, 100, 100)
        cols.append((c.r, c.g, c.b))
    return cols

# Metric 1: tổng quãng đường di chuyển
def compute_total_path_length(trails):
    total = 0.0
    for trail in trails:
        for i in range(1, len(trail)):
            dx = trail[i][0] - trail[i-1][0]
            dy = trail[i][1] - trail[i-1][1]
            total += math.hypot(dx, dy)
    return total

# Metric 2: tỷ lệ vùng được bao phủ bởi sensor radius
def compute_coverage_ratio(robots, mask_pts, radius):
    covered = 0
    r2 = radius*radius
    for x,y in mask_pts:
        for r in robots:
            if (x-r.x)**2 + (y-r.y)**2 <= r2:
                covered += 1
                break
    return covered / len(mask_pts)

# Metric 3: hàm chi phí bao phủ J(P)
def compute_coverage_cost(robots, mask_pts, clicked_pt):
    total = 0.0
    wsum = 0.0
    for x,y in mask_pts:
        w = phi(x, y, *clicked_pt)
        d2 = min((x-r.x)**2 + (y-r.y)**2 for r in robots)
        total += w * d2
        wsum += w
    return total / wsum if wsum>0 else float('inf')

# Hàm chính
def main():

    # Chuẩn bị CSV logger
    with open('metrics.csv','w',newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['iteration','path_length','coverage_ratio','coverage_cost'])

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Coverage Control with Voronoi and Evaluation")
    clock = pygame.time.Clock()

    state = 'waiting'
    clicked_pt = None
    density_surf = None
    voronoi_surf = None
    converged = False
    robots = []
    trails = []
    colors = gen_colors(N)
    it = 0

    # tạo surface cho vẽ vùng nhìn được
    view_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

    # Vẽ ban đầu
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
                # vẽ density map
                density_surf = pygame.Surface((WIDTH, HEIGHT))
                density_surf.fill((255,255,255))
                for x,y in full_mask:
                    v = phi(x, y, *clicked_pt)
                    c = int(v * 255)
                    density_surf.set_at((x,y), (c,c,c))
                # khởi robots và trails
                robots = []
                while len(robots) < N:
                    x = random.randint(min_x, max_x)
                    y = random.randint(min_y, max_y)
                    if point_in_poly(x, y, polygon_points):
                        robots.append(Point(x,y))
                trails = [[(r.x, r.y)] for r in robots]
                it = 0
                converged = False

        # Vẽ background
        if density_surf:
            screen.blit(density_surf, (0,0))
        else:
            screen.fill((255,255,255))
        pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)

        # Lloyd iteration
        if state=='running' and not converged and it<max_iters:
            regions = {i: [] for i in range(N)}
            for x,y in mask_ds:
                j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                regions[j].append((x,y))
            newr = []
            moves = []
            for i in range(N):
                pts = regions[i]
                if not pts:
                    newr.append(robots[i])
                    moves.append(0)
                else:
                    ws=xs=ys=0.0
                    for x,y in pts:
                        w = phi(x,y,*clicked_pt)
                        ws += w; xs += w*x; ys += w*y
                    cx, cy = xs/ws, ys/ws
                    nx = robots[i].x + alpha*(cx-robots[i].x)
                    ny = robots[i].y + alpha*(cy-robots[i].y)
                    newr.append(Point(nx, ny))
                    moves.append(math.hypot(nx-robots[i].x, ny-robots[i].y))
                trails[i].append((newr[i].x, newr[i].y))
            max_move = max(moves)
            robots = newr
            it += 1
            total_dist  = compute_total_path_length(trails)
            cov_ratio   = compute_coverage_ratio(robots, mask_ds, view_radius)
            cov_cost    = compute_coverage_cost(robots, mask_ds, clicked_pt)
            writer.writerow([it,total_dist,cov_ratio,cov_cost])
            
            if max_move < tol:
                converged = True
                print(f"Hội tụ sau {it} bước, max_move={max_move:.4f}")
                # Tính metrics
                # total_dist  = compute_total_path_length(trails)
                # cov_ratio   = compute_coverage_ratio(robots, mask_ds, view_radius)
                # cov_cost    = compute_coverage_cost(robots, mask_ds, clicked_pt)
                print(f"Tổng đường đi: {total_dist:.2f}")
                print(f"Coverage ratio (R={view_radius}): {cov_ratio:.3f}")
                print(f"Coverage cost J: {cov_cost:.3f}")
                # tạo Voronoi overlay
                voronoi_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                for x,y in full_mask:
                    j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                    rcol = colors[j]
                    voronoi_surf.set_at((x,y), (*rcol, 80))

        # Sau khi hội tụ: vẽ Voronoi và vùng nhìn được
        if converged and voronoi_surf:
            screen.blit(voronoi_surf, (0,0))
            view_surf.fill((0,0,0,0))
            for rcol, r in zip(colors, robots):
                pygame.draw.circle(view_surf, (*rcol, 50), (int(r.x), int(r.y)), view_radius)
            screen.blit(view_surf, (0,0))

        # Vẽ trails + robots
        for i, trail in enumerate(trails):
            if len(trail)>1:
                pygame.draw.lines(screen, colors[i], False, trail, 2)
            if robots:
                pygame.draw.circle(screen, colors[i], (int(robots[i].x), int(robots[i].y)), 6)

        # Vẽ click center
        if clicked_pt:
            pygame.draw.circle(screen, (255,255,255), clicked_pt, 8)
            pygame.draw.circle(screen, (0,0,0), clicked_pt, 8, 2)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()

if __name__=="__main__":
    main()