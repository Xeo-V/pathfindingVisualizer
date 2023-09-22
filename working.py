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

    if current != end:
        return [] 
    shortest_path = []
    while current != start:
        shortest_path.append(current)
        current = path[current]
    shortest_path.reverse()
    return shortest_path



def heuristic(a, b):
    return sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def a_star(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    open_set = [(0, start)] 
    g_score = {start: 0} 
    f_score = {start: heuristic(start, end)} 
    came_from = {}  
    
    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                yield nr, nc
                
    while open_set:
        current = heapq.heappop(open_set)[1]
        if current == end:
           
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path[::-1] 
        for neighbor in neighbors(*current):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return [] 

def bfs(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    visited = [[False] * cols for _ in range(rows)]
    path = {}
    
    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc] and not visited[nr][nc]:
                yield nr, nc
                
    queue = deque([start])
    visited[start[0]][start[1]] = True
    
    while queue:
        current = queue.popleft()
        if current == end:
            
            path_list = []
            while current in path:
                path_list.append(current)
                current = path[current]
            return path_list[::-1] 
        for neighbor in neighbors(*current):
            queue.append(neighbor)
            visited[neighbor[0]][neighbor[1]] = True
            path[neighbor] = current
    return []

def dfs(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    visited = [[False] * cols for _ in range(rows)]
    path = {}

    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc] and not visited[nr][nc]:
                yield nr, nc

    stack = [(start, None)] 

    while stack:
        current, predecessor = stack.pop()
        if visited[current[0]][current[1]]:
            continue
        visited[current[0]][current[1]] = True
        path[current] = predecessor
        if current == end:
            break
        stack.extend((neighbor, current) for neighbor in neighbors(*current))

    if current != end:
        return [] 

   
    path_list = []
    while current != start:
        path_list.append(current)
        current = path[current]
    return path_list[::-1] 
