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
scale = 200.0      # tham số Gauss
N = 8              # số robot
alpha = 0.3        # step‐size Lloyd
max_iters = 100    # số bước tối đa
# --------------------------------

Point = namedtuple('Point',['x','y'])

def point_in_poly(x, y, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[(i+1)%n]
        if ((yi>y) != (yj>y)) and (x < (xj-xi)*(y-yi)/(yj-yi) + xi):
            inside = not inside
    return inside

# Tính bounding‐box để giảm vùng tính mask
min_x = min(p[0] for p in polygon_points)
max_x = max(p[0] for p in polygon_points)
min_y = min(p[1] for p in polygon_points)
max_y = max(p[1] for p in polygon_points)

# 1) Precompute mask các pixel nằm trong polygon
mask = [
    (x,y)
    for x in range(min_x, max_x+1)
    for y in range(min_y, max_y+1)
    if point_in_poly(x,y,polygon_points)
]

def phi(x, y, cx, cy):
    dx = (x-cx)/scale
    dy = (y-cy)/scale
    return math.exp(-(dx*dx + dy*dy))

def gen_colors(n):
    """Sinh n màu khác biệt bằng HSV đều nhau."""
    cols = []
    for i in range(n):
        c = pygame.Color(0)
        c.hsva = (i*360/n, 100, 100, 100)
        cols.append((c.r, c.g, c.b))
    return cols

def main():

    with open('metrics.csv','w',newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['iteration','path_length','coverage_ratio','coverage_cost'])

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Coverage Control with Trails")
    clock = pygame.time.Clock()

    clicked_pt = None
    state = 'waiting'   # 'waiting' → chờ click, 'running' → chạy Lloyd
    robots = []
    trails = []         # trails[i] là list các vị trí qua của robot i
    colors = gen_colors(N)
    iter_count = 0

    # vẽ nền + polygon ban đầu
    screen.fill((255,255,255))
    pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)
    pygame.display.flip()

    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running=False
            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button==1 and state=='waiting':
                # 1) nhận tâm, chuyển state
                clicked_pt = ev.pos
                state = 'running'
                # 2) khởi tạo robots ngẫu nhiên trong polygon
                robots = []
                while len(robots) < N:
                    x = random.randint(min_x, max_x)
                    y = random.randint(min_y, max_y)
                    if point_in_poly(x,y,polygon_points):
                        robots.append(Point(x,y))
                # 3) khởi trails
                trails = [[(r.x, r.y)] for r in robots]
                iter_count = 0

        # --- VẼ MỖI FRAME ---
        screen.fill((255,255,255))

        if clicked_pt:
            # 4) background density với 40% opacity
            for x,y in mask:
                val = phi(x,y, *clicked_pt)
                c = int(val * 255 * 0.4)
                screen.set_at((x,y), (c,c,c))

        # 5) vẽ polygon outline
        pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)

        if state == 'running' and iter_count < max_iters:
            # --- 1 bước Lloyd ---
            # a) phân vùng Voronoi rời rạc
            regions = {i: [] for i in range(N)}
            for x,y in mask:
                j = min(range(N),
                        key=lambda i: (x-robots[i].x)**2 + (y-robots[i].y)**2)
                regions[j].append((x,y))
            # b) tính centroid có trọng số φ
            newrs = []
            for i in range(N):
                pts = regions[i]
                if not pts:
                    newrs.append(robots[i])
                    continue
                wsum = xs = ys = 0.0
                for x,y in pts:
                    w = phi(x,y, *clicked_pt)
                    wsum += w; xs += w*x; ys += w*y
                cx = xs/wsum; cy = ys/wsum
                # c) di chuyển với α
                nx = robots[i].x + alpha*(cx - robots[i].x)
                ny = robots[i].y + alpha*(cy - robots[i].y)
                newrs.append(Point(nx, ny))
                # lưu trail
                trails[i].append((nx, ny))
            robots = newrs
            iter_count += 1

        # 6) vẽ trails và robots
        for i, trail in enumerate(trails):
            if len(trail) > 1:
                pygame.draw.lines(screen, colors[i], False, trail, 2)
        for i, r in enumerate(robots):
            if clicked_pt:
                pygame.draw.circle(screen, colors[i],
                                   (int(r.x), int(r.y)), 6)

        # 7) vẽ tâm click (mầu trắng viền đen)
        if clicked_pt:
            pygame.draw.circle(screen, (255,255,255), clicked_pt, 8)
            pygame.draw.circle(screen, (0,0,0), clicked_pt, 8, 2)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
