import heapq

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

# Heuristic values (estimated for demonstration purposes)
heuristics = {
    'S': 15,
    'A': 7,
    'B': 10,
    'C': 5,
    'D': 4,
    'E': 7,
    'F': 9,
    'G1': 0,  # Goal state
    'G2': 0,  # Goal state
    'H': 6,
    'J': 8
}
class Node:
    def __init__(self, name, h=0):
        self.name = name
        self.h = h  # Heuristic to goal

    def __lt__(self, other):
        return self.h < other.h

def greedy_best_first_search(start, goals, graph, heuristics):
    open_list = []
    closed_list = set()
    
    start_node = Node(start, h=heuristics[start])
    heapq.heappush(open_list, (start_node.h, start_node))

    parent_map = {start: None}  # To keep track of the path

    print(f"Starting Greedy Best-First Search from '{start}' to reach one of the goals {goals}.\n")

    while open_list:
        _, current = heapq.heappop(open_list)

        # Print current state
        print(f"Exploring node '{current.name}' with h = {current.h}")

        # Check if we reached a goal state
        if current.name in goals:
            print("\nGoal reached! Reconstructing path...")
            path = []
            while current:
                path.append(current.name)
                current = parent_map[current.name]
            return path[::-1]  # Return reversed path to get start-to-goal order

        closed_list.add(current.name)

        # Expand neighbors
        for neighbor, _ in graph[current.name].items():
            if neighbor in closed_list:
                continue
            
            h_cost = heuristics[neighbor]
            neighbor_node = Node(neighbor, h=h_cost)

            # Print step for each neighbor
            print(f"  Neighbor '{neighbor}' with h = {h_cost}")

            if any(open_node.name == neighbor and open_node.h <= h_cost for _, open_node in open_list):
                continue
            
            parent_map[neighbor] = current
            heapq.heappush(open_list, (h_cost, neighbor_node))

    print("No path found to the goal.")
    return None

# Run the Greedy Best-First Search algorithm
start = 'S'
goals = {'G1', 'G2'}
path = greedy_best_first_search(start, goals, graph, heuristics)

# Output the final path
if path:
    print("\nPath found:", " -> ".join(path))
else:
    print("\nNo path found.")
