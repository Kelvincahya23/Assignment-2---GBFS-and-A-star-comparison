import heapq
import time
from queue import PriorityQueue

goal_state = (1, 2, 3,
              4, 5, 6,
              7, 8, 0)

def find_zero(state):
    return state.index(0)

def misplaced_tiles(state):
    return sum([1 for i in range(9) if state[i] != 0 and state[i] != goal_state[i]])


def get_neighbors(state):
    neighbors = []
    index = find_zero(state)
    row, col = divmod(index, 3)
    directions = {
        'Up': (row - 1, col),
        'Down': (row + 1, col),
        'Left': (row, col - 1),
        'Right': (row, col + 1)
    }

    for move, (r, c) in directions.items():
        if 0 <= r < 3 and 0 <= c < 3:
            new_index = r * 3 + c
            new_state = list(state)
            new_state[index], new_state[new_index] = new_state[new_index], new_state[index]
            neighbors.append(tuple(new_state))

    return neighbors


def gbfs(start):
    start_time = time.time()
    frontier = [(misplaced_tiles(start), start)]
    visited = set()
    came_from = {}
    nodes_expanded = 0

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal_state:
            break

        visited.add(current)
        nodes_expanded += 1

        for neighbor in get_neighbors(current):
            if neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(frontier, (misplaced_tiles(neighbor), neighbor))

    path = reconstruct_path(came_from, start, goal_state)
    return path, time.time() - start_time, nodes_expanded


def astar(start):
    start_time = time.time()
    frontier = [(misplaced_tiles(start), 0, start)]
    visited = set()
    came_from = {}
    g_score = {start: 0}
    nodes_expanded = 0

    while frontier:
        _, g, current = heapq.heappop(frontier)

        if current == goal_state:
            break

        visited.add(current)
        nodes_expanded += 1

        for neighbor in get_neighbors(current):
            temp_g = g + 1
            if neighbor not in g_score or temp_g < g_score[neighbor]:
                g_score[neighbor] = temp_g
                f = temp_g + misplaced_tiles(neighbor)
                came_from[neighbor] = current
                heapq.heappush(frontier, (f, temp_g, neighbor))

    path = reconstruct_path(came_from, start, goal_state)
    return path, time.time() - start_time, nodes_expanded


def reconstruct_path(came_from, start, end):
    path = [end]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def print_path(path):
    for state in path:
        print("-" * 7)
        for i in range(0, 9, 3):
            print(state[i:i+3])
    print("-" * 7)


initial_board = (1, 2, 3,
                 4, 0, 6,
                 7, 5, 8)

print("=== Greedy Best-First Search ===")
gbfs_path, gbfs_time, gbfs_nodes = gbfs(initial_board)
print_path(gbfs_path)
print(f"Steps: {len(gbfs_path)-1}, Time: {gbfs_time:.4f}s, Nodes Expanded: {gbfs_nodes}\n")

print("=== A* Search ===")
astar_path, astar_time, astar_nodes = astar(initial_board)
print_path(astar_path)
print(f"Steps: {len(astar_path)-1}, Time: {astar_time:.4f}s, Nodes Expanded: {astar_nodes}")
