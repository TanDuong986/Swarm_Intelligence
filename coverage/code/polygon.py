import pygame
import math
import sys

# --------- CẤU HÌNH CHÍNH ---------
WIDTH, HEIGHT = 1200, 700
# Danh sách đỉnh của polygon lồi (theo thứ tự nối vòng)
polygon_points = [
    (50, 650),
    (1100, 600),
    (1000, 50),
    (600, 20),
    (60, 60)
]
# Tham số điều khiển "độ lan tỏa" của Gauss trên màn hình
scale = 200.0
# -----------------------------------
def point_in_poly(pt, poly):
    x, y = pt
    inside = False
    n = len(poly)
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[(i+1) % n]
        if ((yi > y) != (yj > y)) and (x < (xj-xi)*(y-yi)/(yj-yi) + xi):
            inside = not inside
    return inside

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Gaussian on Convex Polygon")
    clock = pygame.time.Clock()

    clicked_pt = None
    # khởi tạo: nền trắng, chỉ vẽ viền polygon
    screen.fill((255,255,255))
    pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)
    pygame.display.flip()

    while True:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
                clicked_pt = ev.pos
                # làm lại nền trắng
                screen.fill((255,255,255))
                # tính và vẽ Gauss: invert val để tâm là 0 (đen), xa tâm → 1 (trắng)
                for x in range(WIDTH):
                    for y in range(HEIGHT):
                        if point_in_poly((x,y), polygon_points):
                            dx = (x - clicked_pt[0]) / scale
                            dy = (y - clicked_pt[1]) / scale
                            val = math.exp(-(dx*dx + dy*dy))
                            c = int((1 - val) * 255)
                            screen.set_at((x,y), (c,c,c))
                # vẽ viền polygon (đen) và tâm (trắng)
                pygame.draw.polygon(screen, (0,0,0), polygon_points, 2)
                pygame.draw.circle(screen, (255,255,255), clicked_pt, 5)
                pygame.display.flip()

        clock.tick(30)

if __name__ == "__main__":
    print("Polygon vertices:", polygon_points)
    main()