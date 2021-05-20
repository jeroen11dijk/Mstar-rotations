import heapq
import itertools

from Astar import Astar, Node


class MstarNode:
    def __init__(self, nodes, cost=float('inf')):
        """
        Used as the main datastructure
        nodes: is a list of nodes to store the Astar node per agent
        cost: is the cost to reach this node
        backset: is the set of MstarNodes that have been explored to get to this MstarNode
        backptr: the parent of the node
        collision_set: The agents that are/will be in a collision if they follow their optimal path
        """
        self.nodes = nodes
        self.cost = cost
        self.backset = set()
        self.backptr = None
        self.collision_set = set()

    def __eq__(self, other):
        return self.nodes == other.nodes

    def __lt__(self, other):
        """ Used to do the sorting of the heap, by comparing the sum of the heuristics per node. """
        return sum([node.h for node in self.nodes]) < sum([node.h for node in other.nodes])

    def __repr__(self):
        return '({0}, {1})'.format(self.nodes, self.cost)

    def __hash__(self):
        return hash(tuple([node for node in self.nodes]))


class Mstar:
    def __init__(self, grid, start, end):
        self.grid = grid
        self.n_agents = len(start)
        self.policy = {}
        # Convert the start and end to a MstarNode structure
        current = []
        self.goals = []
        for i in range(self.n_agents):
            self.goals.append(Node((end[i][0], end[i][1]), None, end[i][2], 0))
            current.append(
                Node((start[i][0], start[i][1]), None, start[i][2], self.heuristic(i, start[i], start[i][2])))
        self.start = MstarNode(current)
        self.goal = MstarNode(self.goals)
        # Create the heap and push the starting note to it
        self.open = []
        self.seen = {MstarNode(current, 0): MstarNode(current, 0)}
        heapq.heappush(self.open, MstarNode(current, 0))

    def solve(self):
        while len(self.open) > 0:
            # Get the MstarNode with the lowest heuristic
            current = heapq.heappop(self.open)
            # If this is the goal then retrace the path and return it
            if current == self.goal:
                res = []
                while current != self.start:
                    res.append(current.nodes)
                    current = current.backptr
                return res
            # Otherwise we loop over all the neighbors
            neighbors = self.getNeighbors(current)
            for nbr in neighbors:
                # Convert the neighbor to a MstarNode and add current to its backset
                if MstarNode(nbr) in self.seen:
                    neighbor = self.seen[MstarNode(nbr)]
                else:
                    neighbor = MstarNode(nbr)
                    self.seen[neighbor] = neighbor
                neighbor.backset.add(current)
                # Look if there are collisions in neighbor and add those to the collision set and backpropagate this
                newCollisions = self.phi(current.nodes, neighbor.nodes)
                neighbor.collision_set.update(newCollisions)
                self.backprop(current, neighbor.collision_set)
                # If neighbor has no collisions and the current path to it is cheaper
                if len(newCollisions) == 0 and current.cost + self.getMoveCost(current, neighbor) < neighbor.cost:
                    # Update the cost, add current to its backptr and push it to the heap
                    neighbor.cost = current.cost + self.getMoveCost(current, neighbor)
                    neighbor.backptr = current
                    heapq.heappush(self.open, neighbor)
        raise ValueError("No path has found")

    def getMoveCost(self, current, next):
        """"Gets the move cost between two MstarNodes"""
        cost = 0
        for i in range(self.n_agents):
            if current.nodes[i] != next.nodes[i]:
                cost += 1
        return cost

    def backprop(self, current: MstarNode, collisions):
        """"Backpropagate a new collision to all backsets recursively"""
        if not collisions.issubset(current.collision_set):
            current.collision_set.update(collisions)
            heapq.heappush(self.open, current)
            for next in current.backset:
                self.backprop(next, current.collision_set)

    def phi(self, current, next):
        """
        Looks for collisions between the current and next MstarNode.
        There are two types, namely two agents in current being on the same vertex,
        or an edge being used twice between current and next.
        """
        current = [temp.position for temp in current]
        next = [temp.position for temp in next]
        seen = set()
        double = set()
        edges = {k: l for k, l in zip(current, next)}
        for i, val in enumerate(next):
            if val in seen or val != current[i] and val in edges and edges[val] == current[i]:
                double.add(val)
            else:
                seen.add(val)
        return set([i for i, node in enumerate(next) if node in double])

    def getNeighbors(self, current: MstarNode):
        """Get the neighbors of a MstarNode"""
        neighbors = []
        options = []
        # Loop over all the agents
        for i in range(self.n_agents):
            node: Node = current.nodes[i]
            options_i = []
            if i in current.collision_set:
                # If the agent in the collision set we add the current node as well as all possible nodes
                options_i.append(node)
                (x, y) = node.position
                moves = {0: (x - 1, y), 90: (x, y + 1), 180: (x + 1, y), 270: (x, y - 1)}
                options_i.append(Node(node.position, node, node.rotation + 90, node.h))
                options_i.append(Node(node.position, node, node.rotation - 90, node.h))
                if self.grid[moves[node.rotation][0]][moves[node.rotation][1]] == 0:
                    options_i.append(Node(moves[node.rotation], node, node.rotation,
                                          self.heuristic(i, moves[node.rotation], node.rotation)))
            else:
                # If the agent is not in the collision set we add only the optimal following node
                try:
                    if (node, self.goal.nodes[i]) in self.policy:
                        nextPos = self.policy[(node, self.goal.nodes[i])]
                    else:
                        nextPos = Astar(self.grid, node, self.goal.nodes[i]).solve()
                        self.policy[(node, self.goal.nodes[i])] = nextPos
                except ValueError:
                    print(f"start: {node}, goal: {self.goal.nodes[i]}")
                    raise RuntimeError()
                options_i.append(Node(nextPos[0], node, nextPos[1], self.heuristic(i, nextPos[0], nextPos[1])))
            options.append(options_i)
        # Take the cartesian product to get all options
        for element in itertools.product(*options):
            neighbors.append(list(element))
        return neighbors

    def heuristic(self, index, position, rotation):
        return abs(position[0] - self.goals[index].position[0]) + abs(
            position[1] - self.goals[index].position[1]) + + abs((rotation - self.goals[index].rotation) / 90)


if __name__ == "__main__":
    matrix = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]
    start = (
        (23, 21, 90), (31, 9, 90), (5, 9, 0), (12, 11, 90), (1, 24, 180), (27, 23, 180), (28, 9, 180),
        (3, 7, 0), (5, 21, 0), (30, 23, 90), (20, 9, 180))
    end = ((26, 3, 180), (45, 20, 0), (44, 9, 0), (24, 17, 270), (37, 13, 270), (22, 7, 0), (3, 4, 0),
           (31, 17, 90), (16, 25, 180), (10, 21, 180), (16, 23, 0))
    print(Mstar(matrix, start, end).solve())
