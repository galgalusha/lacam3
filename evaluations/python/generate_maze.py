import random

def generate_maze_with_rooms(
    size=65,
    num_rooms=6,
    room_size=3,  # 3x3 or 5x5 works well for passing bays
    seed=42,
    filename="maze-65-65-rooms.map",
):
    assert size % 2 == 1, "Use an odd size (e.g., 51 or 65)."
    random.seed(seed)
    grid = [["@" for _ in range(size)] for _ in range(size)]

    # 1. Generate 1-cell-wide DFS maze
    stack = [(1, 1)]
    grid[1][1] = "."
    while stack:
        r, c = stack[-1]
        dirs = [(-2, 0), (2, 0), (0, -2), (0, 2)]
        random.shuffle(dirs)
        moved = False
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 1 <= nr < size - 1 and 1 <= nc < size - 1 and grid[nr][nc] == "@":
                grid[r + dr // 2][c + dc // 2] = "."
                grid[nr][nc] = "."
                stack.append((nr, nc))
                moved = True
                break
        if not moved:
            stack.pop()

    # 2. Carve a few small rooms on odd-aligned coordinates
    odd_coords = list(range(1, size - room_size, 2))
    for _ in range(num_rooms):
        r0 = random.choice(odd_coords)
        c0 = random.choice(odd_coords)
        for r in range(r0, r0 + room_size):
            for c in range(c0, c0 + room_size):
                grid[r][c] = "."

    with open(filename, "w") as f:
        f.write(f"type octile\nheight {size}\nwidth {size}\nmap\n")
        for row in grid:
            f.write("".join(row) + "\n")

generate_maze_with_rooms(size=51, num_rooms=40, room_size=3, seed=42)