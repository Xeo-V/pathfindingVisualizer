import heapq
from math import sqrt
from collections import deque
import heapq
import random

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


def swarm(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    visited = [[False] * cols for _ in range(rows)]
    distance = [[float('inf')] * cols for _ in range(rows)]
    distance[start[0]][start[1]] = 0
    path = {}
    
    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                yield nr, nc
    
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    pq = [(0, start)]
    while pq:
        _, current = heapq.heappop(pq)
        if visited[current[0]][current[1]]:
            continue
        visited[current[0]][current[1]] = True
        if current == end:
            break
        for neighbor in neighbors(*current):
            alt = distance[current[0]][current[1]] + 1
            if alt < distance[neighbor[0]][neighbor[1]]:
                distance[neighbor[0]][neighbor[1]] = alt
                priority = alt + heuristic(end, neighbor)
                heapq.heappush(pq, (priority, neighbor))
                path[neighbor] = current
    
    if not visited[end[0]][end[1]]:
        return []
    path_list = []
    while current != start:
        path_list.append(current)
        current = path[current]
    return path_list[::-1]


def greedy_best_first(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    visited = [[False] * cols for _ in range(rows)]
    path = {}
    
    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                yield nr, nc
    
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    pq = [(heuristic(start, end), start)]
    while pq:
        _, current = heapq.heappop(pq)
        if visited[current[0]][current[1]]:
            continue
        visited[current[0]][current[1]] = True
        if current == end:
            break
        for neighbor in neighbors(*current):
            if not visited[neighbor[0]][neighbor[1]]:
                heapq.heappush(pq, (heuristic(end, neighbor), neighbor))
                path[neighbor] = current
    
    if not visited[end[0]][end[1]]:
        return []
    path_list = []
    while current != start:
        path_list.append(current)
        current = path[current]
    return path_list[::-1]

def bidirectional_search(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    visited_start = [[False] * cols for _ in range(rows)]
    visited_end = [[False] * cols for _ in range(rows)]
    path_start = {}
    path_end = {}
    
    def neighbors(r, c):
        for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not grid[nr][nc]:
                yield nr, nc
    
    def meet_point():
        for r in range(rows):
            for c in range(cols):
                if visited_start[r][c] and visited_end[r][c]:
                    return (r, c)
        return None
    
    frontier_start = [start]
    frontier_end = [end]
    while frontier_start and frontier_end:
        current_start = frontier_start.pop(0)
        visited_start[current_start[0]][current_start[1]] = True
        for neighbor in neighbors(*current_start):
            if not visited_start[neighbor[0]][neighbor[1]]:
                frontier_start.append(neighbor)
                path_start[neighbor] = current_start
        
        current_end = frontier_end.pop(0)
        visited_end[current_end[0]][current_end[1]] = True
        for neighbor in neighbors(*current_end):
            if not visited_end[neighbor[0]][neighbor[1]]:
                frontier_end.append(neighbor)
                path_end[neighbor] = current_end
        
        meet = meet_point()
        if meet:
            
            path_list = []
            current = meet
            while current != start:
                path_list.append(current)
                current = path_start[current]
            path_list.reverse()
            current = meet
            while current != end:
                current = path_end[current]
                path_list.append(current)
            return path_list
    return []

class Ant:
    def __init__(self, start):
        self.current_node = start
        self.path = [start]
        self.path_length = 0

def initialize_pheromones(graph):
    pheromones = {}
    for node in graph:
        pheromones[node] = {neighbor: 1 for neighbor in graph[node]}
    return pheromones

def move_ant(ant, graph, pheromones, alpha, beta):
    current_node = ant.current_node
    neighbors = set(graph[current_node]) - set(ant.path)
    
    if not neighbors:
        return False  # No available moves
    
    probabilities = []
    for neighbor in neighbors:
        pheromone = pheromones[current_node][neighbor]
        heuristic = 1 / graph[current_node][neighbor]  # Assuming graph stores distances
        probabilities.append((pheromone ** alpha) * (heuristic ** beta))
    
    chosen_neighbor = random.choices(list(neighbors), weights=probabilities)[0]
    ant.path.append(chosen_neighbor)
    ant.path_length += graph[current_node][chosen_neighbor]
    ant.current_node = chosen_neighbor
    return True

def update_pheromones(ant, pheromones, evaporation_rate):
    for i in range(len(ant.path) - 1):
        current_node = ant.path[i]
        next_node = ant.path[i + 1]
        pheromones[current_node][next_node] = (1 - evaporation_rate) * pheromones[current_node][next_node] + 1 / ant.path_length

def ant_colony_optimization(graph, start, end, num_ants, num_iterations, alpha, beta, evaporation_rate):
    pheromones = initialize_pheromones(graph)
    best_path = None
    best_length = float('inf')
    
    for _ in range(num_iterations):
        ants = [Ant(start) for _ in range(num_ants)]
        
        for ant in ants:
            while ant.current_node != end:
                move_ant(ant, graph, pheromones, alpha, beta)
            update_pheromones(ant, pheromones, evaporation_rate)
            
            if ant.path_length < best_length:
                best_path = ant.path
                best_length = ant.path_length
    
    return best_path
