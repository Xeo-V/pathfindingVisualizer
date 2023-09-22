import heapq
from math import sqrt
from collections import deque

def dijkstra(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    distances = [[float('inf')] * cols for _ in range(rows)]
    distances[start[0]][start[1]] = 0
    visited = [[False] * cols for _ in range(rows)]
    path = {}

    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc] and not visited[nr][nc]:
                yield nr, nc

    while True:
        min_distance = float('inf')
        current = None
        for r in range(rows):
            for c in range(cols):
                if not visited[r][c] and distances[r][c] < min_distance:
                    min_distance = distances[r][c]
                    current = (r, c)
        if current is None or current == end:
            break
        r, c = current
        visited[r][c] = True
        for nr, nc in neighbors(r, c):
            alt_distance = distances[r][c] + 1
            if alt_distance < distances[nr][nc]:
                distances[nr][nc] = alt_distance
                path[(nr, nc)] = current

    # Reconstruct the shortest path
    if current != end:
        return []  # No path found
    shortest_path = []
    while current != start:
        shortest_path.append(current)
        current = path[current]
    shortest_path.reverse()
    return shortest_path


def a_star(grid, start, end):
    # A* algorithm logic
    pass

#other pathfinding algorithms
    
