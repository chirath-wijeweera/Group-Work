import heapq

def heuristic(a, b):
    """Calculate the Manhattan distance between two points"""
    return abs(b[0] - a[0]) + abs(b[1] - a[1])

def a_star_search(maze, start, end):
    """Perform A* search from start to end in a maze."""
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Directions: right, down, left, up
    close_set = set()  # Nodes that have already been evaluated
    came_from = {}  # Track the path
    gscore = {start: 0}  # Cost from start to current node
    fscore = {start: heuristic(start, end)}  # Estimated total cost from start to end
    oheap = []  # Open heap queue

    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == end:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]  # Return the path in the correct order from start to end

        close_set.add(current)
        for i, j in neighbors:
            neighbor = (current[0] + i, current[1] + j)
            if 0 <= neighbor[0] < len(maze) and 0 <= neighbor[1] < len(maze[0]) and maze[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = gscore[current] + 1  # Assumes cost from one node to another is 1
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                    continue
                if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

maze = [
    [0, 0, 0, 0, 1, 0],
    [1, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0, 0]
]

start = (0, 0)  # Starting node
end = (4, 5)  # Ending node (goal)

path = a_star_search(maze, start, end)
print("Path from start to end:", path)
