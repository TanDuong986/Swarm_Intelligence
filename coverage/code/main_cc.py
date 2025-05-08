import pygame, sys, math, random
from collections import namedtuple
import csv

# -------- CẤU HÌNH CHÍNH --------
WIDTH, HEIGHT    = 1200, 700
polygon_points   = [
    (50, 650),
    (1100, 600),
    (1000, 50),
    (600, 20),
    (60, 60)
]
scale       = 200.0   # Gauss spread
N           = 10       # số robot
alpha       = 0.2     # Lloyd step-size
max_iters   = 100     # số bước tối đa
tol         = 3.0     # ngưỡng hội tụ (pixel)
down_samp   = 5       # down-sampling cho Voronoi
view_radius = 100     # bán kính cảm biến (pixel)
phi_eps = 0.01        # giá trị nhỏ nhất của phi(x,y,cx,cy) để lấy làm vùng tìm kiếm 


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
    print(len(mask_pts))
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
    wsum  = 0.0
    for x,y in mask_pts:
        w   = phi(x, y, *clicked_pt)
        d2  = min((x-r.x)**2 + (y-r.y)**2 for r in robots)
        total += w * d2
        wsum  += w
    return total / wsum if wsum>0 else float('inf')

def main():
    # Mở file CSV một lần, giữ đến khi kết thúc
    csv_file = open('metrics.csv','w', newline='')
    writer   = csv.writer(csv_file)
    writer.writerow(['iteration','path_length','coverage_ratio','coverage_cost'])
    
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Coverage Control with Voronoi and Evaluation")
    clock = pygame.time.Clock()

    state      = 'waiting'
    clicked_pt = None
    density_surf = voronoi_surf = None
    converged  = False
    robots     = []
    trails     = []
    colors     = gen_colors(N)
    it         = 0

    # tạo surface cho vẽ vùng nhìn được
    view_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)

    # Vẽ khung ban đầu
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

                mask_rescue = [
                    (x, y) for (x, y) in full_mask if phi(x, y, *clicked_pt) > phi_eps
                ]
                mask_rescue_ds = mask_rescue[::down_samp]
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

            newr  = []
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

            robots = newr
            it += 1


            # Tính metrics
            total_dist = compute_total_path_length(trails)
            cov_ratio  = compute_coverage_ratio(robots, mask_rescue_ds, view_radius)
            cov_cost   = compute_coverage_cost(robots, mask_ds, clicked_pt)

            # Ghi vào CSV và flush ngay để tránh mất dữ liệu
            writer.writerow([it, total_dist, cov_ratio, cov_cost])
            csv_file.flush()

            # Kiểm tra hội tụ
            if max(moves) < tol:
                converged = True
                print(f"Hội tụ sau {it} bước, max_move={max(moves):.4f}")
                print(f"Tổng đường đi: {total_dist:.2f}")
                print(f"Coverage ratio (R={view_radius}): {cov_ratio:.3f}")
                print(f"Coverage cost J: {cov_cost:.3f}")
                # Tạo Voronoi overlay
                voronoi_surf = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                for x,y in full_mask:
                    j = min(range(N), key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                    rcol = colors[j]
                    voronoi_surf.set_at((x,y), (*rcol, 80))

        # Khi đã hội tụ, vẽ Voronoi và vùng nhìn
        if converged and voronoi_surf:
            screen.blit(voronoi_surf, (0,0))
            view_surf.fill((0,0,0,0))
            for rcol, r in zip(colors, robots):
                pygame.draw.circle(view_surf, (*rcol, 50), (int(r.x), int(r.y)), view_radius)
            screen.blit(view_surf, (0,0))

        # Vẽ trails và robots
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

    # Đóng file CSV khi thoát
    csv_file.close()
    pygame.quit()
    sys.exit()

if __name__=="__main__":
    main()
