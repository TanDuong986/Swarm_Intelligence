import pygame
import json
import os

def read_map_from_json(json_file):
    """
    Đọc dữ liệu map từ file JSON.
    File JSON có thể chứa dữ liệu trực tiếp dưới dạng ma trận 2D 
    hoặc được lưu dưới key "data".
    :param json_file: tên file JSON cần đọc.
    :return: ma trận 2D (danh sách các danh sách) chứa các giá trị 0 và 1.
    """
    if not os.path.exists(json_file):
        raise FileNotFoundError(f"File {json_file} không tồn tại.")
    with open(json_file, 'r', encoding='utf-8') as f:
        data = json.load(f)
    # Nếu dữ liệu nằm dưới key "data", lấy phần đó
    if isinstance(data, dict):
        map_data = data.get("data")
    else:
        map_data = data
    return map_data

def draw_grid_map(map_data, screen, cell_size):
    """
    Vẽ grid map dựa trên ma trận map_data.
    Ô có giá trị 0 sẽ vẽ màu trắng, ô có giá trị 1 sẽ vẽ màu đen.
    Các đường viền của ô sẽ được vẽ bằng màu xám.
    :param map_data: ma trận 2D của map.
    :param screen: surface của Pygame để vẽ.
    :param cell_size: kích thước của mỗi ô (pixel).
    """
    rows = len(map_data)
    cols = len(map_data[0])
    for i in range(rows):
        for j in range(cols):
            # 0: ô trống (trắng), 1: vật cản (đen)
            color = (255, 255, 255) if map_data[i][j] == 0 else (0, 0, 0)
            rect = pygame.Rect(j * cell_size, i * cell_size, cell_size, cell_size)
            pygame.draw.rect(screen, color, rect)
            pygame.draw.rect(screen, (128, 128, 128), rect, 1)

def main():
    pygame.init()
    # Giả sử map của bạn là 20x20, ta đặt cell_size sao cho tổng kích thước là hợp lý (ví dụ 600x600)
    grid_size = 20
    cell_size = 600 // grid_size
    width = cell_size * grid_size
    height = cell_size * grid_size
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Hiển thị Grid Map từ JSON")

    json_file = "map/aStar.json"  # Tên file JSON chứa map (định dạng ma trận 20x20 với giá trị 0,1)
    try:
        map_data = read_map_from_json(json_file)
    except Exception as e:
        print("Lỗi khi đọc map:", e)
        return

    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        draw_grid_map(map_data, screen, cell_size)
        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

if __name__ == "__main__":
    main()
