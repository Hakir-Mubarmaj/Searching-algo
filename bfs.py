from collections import deque

# Graph representation: Each node points to neighbors with edge costs
graph = {
    'S': {'A': 1, 'B': 8, 'C': 4},
    'A': {'D': 5, 'H': 12},
    'B': {'J': 4, 'H': 1, 'G1': 4},
    'C': {'E': 2, 'F': 6},
    'D': {'H': 2},
    'E': {'F': 10},
    'F': {'G2': 2},
    'G1': {},
    'G2': {},
    'H': {'J': 4},
    'J': {'G2': 2}
}

def bfs(start, goals, graph):
    queue = deque([start])
    parent_map = {start: None}  # To reconstruct path
    visited = set([start])

    # Lists to track added and popped elements
    added_elements = [start]
    popped_elements = []

    print(f"Starting Breadth-First Search from '{start}' to reach one of the goals {goals}.\n")

    while queue:
        current = queue.popleft()
        popped_elements.append(current)  # Track the popped element

        # Check if we reached a goal state
        if current in goals:
            print("\nGoal reached! Reconstructing path...")
            path = []
            while current:
                path.append(current)
                current = parent_map[current]
            return path[::-1], added_elements, popped_elements  # Return reversed path, added, and popped lists

        # Expand neighbors
        for neighbor in graph[current].keys():
            if neighbor not in visited:
                parent_map[neighbor] = current
                visited.add(neighbor)
                queue.append(neighbor)
                added_elements.append(neighbor)  # Track the added element

    print("No path found to the goal.")
    return None, added_elements, popped_elements

# Run the Breadth-First Search algorithm
start = 'S'
goals = {'G1', 'G2'}
path, added_elements, popped_elements = bfs(start, goals, graph)

# Output the final path and queue operations
if path:
    print("\nPath found:", " -> ".join(path))
else:
    print("\nNo path found.")

# Print all elements added to the queue and all popped elements
print("\nAll elements added to the queue in order:", added_elements)
print("All elements popped from the queue in order:", popped_elements)
