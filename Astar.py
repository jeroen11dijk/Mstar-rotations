import heapq


class Node:
    # Initialize the class
    def __init__(self, position: (int, int), parent, rotation: int, heuristic: int):
        self.position = position
        self.parent = parent
        self.rotation = rotation % 360
        self.g = 0  # Distance to start node
        self.h = heuristic  # Distance to goal node
        self.f = 0  # Total cost

    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position and self.rotation == other.rotation

    # Sort nodes
    def __lt__(self, other):
        return self.h < other.h

    def __hash__(self):
        return hash((self.position[0], self.position[1], self.h))

    # Print node
    def __repr__(self):
        return ('({0}, {1})'.format(self.position, self.rotation))


class Astar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.goal = goal
        self.start = start

    def heuristic(self, node):
        return abs(node[0] - self.goal.position[0]) + abs(node[1] - self.goal.position[1])

    def cost(self, node):
        return abs(node.position[0] - self.start.position[0]) + abs(node.position[1] - self.start.position[1])

    def solve(self):
        open = []
        closed = set()
        heapq.heappush(open, self.start)
        while len(open) > 0:
            current: Node = heapq.heappop(open)
            closed.add(current)
            if current == self.goal:
                path = []
                while current != self.start:
                    path.append((current.position, current.rotation))
                    current = current.parent
                if len(path) > 0:
                    return path[-1]
                return self.start.position, self.start.rotation

            (x, y) = current.position
            moves = {0: (x - 1, y), 90: (x, y + 1), 180: (x + 1, y), 270: (x, y - 1)}
            turnRight = Node(current.position, current, current.rotation + 90, self.heuristic(current.position))
            turnLeft = Node(current.position, current, current.rotation - 90, self.heuristic(current.position))
            move = Node(moves[current.rotation], current, current.rotation, self.heuristic(moves[current.rotation]))
            neighbors = [turnRight, turnLeft, move]
            for nbr in neighbors:
                xrange = 0 <= nbr.position[0] < len(self.grid)
                yrange = 0 <= nbr.position[1] < len(self.grid[0])
                if not xrange or not yrange or self.grid[nbr.position[0]][nbr.position[1]] == 1:
                    continue
                if nbr in closed:
                    continue
                nbr.g = current.g + 1
                nbr.f = nbr.g + nbr.h
                heapq.heappush(open, nbr)
        return "No path found"
