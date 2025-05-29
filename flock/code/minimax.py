import pygame
import sys

# --- Settings ---
ROWS, COLS = 3, 3
WINDOW_SIZE = 600
CELL_SIZE = WINDOW_SIZE // COLS       # 150 pixels per cell
WIDTH = CELL_SIZE * COLS              # 600 pixels
HEIGHT = CELL_SIZE * ROWS             # 600 pixels
LINE_COLOR = (0, 0, 0)
BG_COLOR = (255, 255, 255)
X_COLOR = (200, 0, 0)
O_COLOR = (0, 0, 200)

# Board: 0 = empty, 1 = X (Human), -1 = O (AI)
board = [[0] * COLS for _ in range(ROWS)]

# Counter for recursion calls
def reset_counter():
    global recursion_counter
    recursion_counter = 0

def increment_counter():
    global recursion_counter
    recursion_counter += 1

reset_counter()

# Draw the grid and pieces
def draw_board(screen):
    screen.fill(BG_COLOR)
    # Draw grid lines
    for i in range(1, ROWS):
        pygame.draw.line(screen, LINE_COLOR, (0, i * CELL_SIZE), (WIDTH, i * CELL_SIZE), 2)
    for j in range(1, COLS):
        pygame.draw.line(screen, LINE_COLOR, (j * CELL_SIZE, 0), (j * CELL_SIZE, HEIGHT), 2)
    # Draw X and O
    for r in range(ROWS):
        for c in range(COLS):
            x = c * CELL_SIZE
            y = r * CELL_SIZE
            if board[r][c] == 1:
                pygame.draw.line(screen, X_COLOR,
                                 (x + 10, y + 10), (x + CELL_SIZE - 10, y + CELL_SIZE - 10), 3)
                pygame.draw.line(screen, X_COLOR,
                                 (x + CELL_SIZE - 10, y + 10), (x + 10, y + CELL_SIZE - 10), 3)
            elif board[r][c] == -1:
                pygame.draw.circle(screen, O_COLOR,
                                   (x + CELL_SIZE // 2, y + CELL_SIZE // 2),
                                   CELL_SIZE // 2 - 10, 3)

# Check 4 in a row for a given player
def check_win(bd, player):
    # horizontal
    for r in range(ROWS):
        if all(bd[r][c] == player for c in range(COLS)):
            return True
    # vertical
    for c in range(COLS):
        if all(bd[r][c] == player for r in range(ROWS)):
            return True
    # diagonal down-right
    if all(bd[i][i] == player for i in range(ROWS)):
        return True
    # diagonal down-left
    if all(bd[i][COLS - 1 - i] == player for i in range(ROWS)):
        return True
    return False

# Basic evaluation: terminal states
def evaluate(bd):
    if check_win(bd, 1):   # Human win
        return 1
    if check_win(bd, -1):  # AI win
        return -1
    # draw or non-terminal
    return 0

# Minimax search with full depth
def minimax(bd, depth, maximizing):
    # count each call
    increment_counter()
    score = evaluate(bd)
    # terminal or depth limit or full board
    if score != 0 or depth == 0 or all(bd[r][c] != 0 for r in range(ROWS) for c in range(COLS)):
        return score
    if maximizing:
        best = -float('inf')
        for r in range(ROWS):
            for c in range(COLS):
                if bd[r][c] == 0:
                    bd[r][c] = 1
                    val = minimax(bd, depth - 1, False)
                    bd[r][c] = 0
                    best = max(best, val)
        return best
    else:
        best = float('inf')
        for r in range(ROWS):
            for c in range(COLS):
                if bd[r][c] == 0:
                    bd[r][c] = -1
                    val = minimax(bd, depth - 1, True)
                    bd[r][c] = 0
                    best = min(best, val)
        return best

# AI selects best move

def ai_move(depth=ROWS*COLS):
    global recursion_counter
    reset_counter()
    best_score = float('inf')
    move = None
    for r in range(ROWS):
        for c in range(COLS):
            if board[r][c] == 0:
                board[r][c] = -1
                score = minimax(board, depth - 1, True)
                board[r][c] = 0
                if score < best_score:
                    best_score = score
                    move = (r, c)
    print(f"Minimax recursive calls: {recursion_counter}")
    return move

# Main game loop
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption('Tic-Tac-Toe 4x4 - Human vs AI')
    human_turn = True

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            # Human (X) move on left-click
            if human_turn and event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                c = mx // CELL_SIZE
                r = my // CELL_SIZE
                if 0 <= r < ROWS and 0 <= c < COLS and board[r][c] == 0:
                    board[r][c] = 1
                    if check_win(board, 1):
                        print("Human wins!")
                        pygame.quit()
                        sys.exit()
                    human_turn = False

        if not human_turn:
            mv = ai_move()
            if mv:
                r, c = mv
                board[r][c] = -1
                if check_win(board, -1):
                    print("AI wins!")
                    pygame.quit()
                    sys.exit()
            human_turn = True

        draw_board(screen)
        pygame.display.flip()

if __name__ == '__main__':
    main()
