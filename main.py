import heapq
import itertools

from Astar import Astar, Node


class MstarNode:
    def __init__(self, nodes, cost=float('inf')):
        self.nodes = nodes
        self.cost = cost
        self.backset = set()
        self.backptr = None
        self.collision_set = set()

    def __eq__(self, other):
        return self.nodes == other.nodes

    def __lt__(self, other):
        return sum([node.h for node in self.nodes]) < sum([node.h for node in other.nodes])

    def __repr__(self):
        return '({0}, {1})'.format(self.nodes, self.cost)

    def __hash__(self):
        return hash(tuple([node for node in self.nodes]))


class Mstar:
    def __init__(self, grid, start, end):
        self.grid = grid
        self.n_agents = len(start)
        current = []
        self.goals = []
        for i in range(self.n_agents):
            self.goals.append(Node((end[i][0], end[i][1]), None, end[i][2], 0))
            current.append(Node((start[i][0], start[i][1]), None, start[i][2], self.heuristic(i, start[i])))
        self.start = MstarNode(current)
        self.goal = MstarNode(self.goals)
        self.open = []
        heapq.heappush(self.open, MstarNode(current, 0))

    def solve(self):
        while len(self.open) > 0:
            current = heapq.heappop(self.open)
            if current == self.goal:
                res = []
                while current != self.start:
                    res.append(current.nodes)
                    current = current.backptr
                return res
            for nbr in self.getNeighbors(current):
                print(nbr)
                neighbor = MstarNode(nbr)
                neighbor.backset.add(current)
                newCollisions = self.phi(neighbor.nodes)
                neighbor.collision_set.update(newCollisions)
                self.backprop(current, neighbor.collision_set)
                if len(newCollisions) == 0 and current.cost + self.getMoveCost(current, neighbor) < neighbor.cost:
                    neighbor.cost = current.cost + self.getMoveCost(current, neighbor)
                    neighbor.backptr = current
                    heapq.heappush(self.open, neighbor)

    def getMoveCost(self, current, next):
        cost = 0
        for i in range(self.n_agents):
            if current.nodes[i] != next.nodes[i]:
                cost += 1
        return cost

    def backprop(self, current: MstarNode, collisions):
        if not collisions.issubset(current.collision_set):
            current.collision_set.update(collisions)
            heapq.heappush(self.open, current)
            for next in current.backset:
                self.backprop(next, current.collision_set)

    def phi(self, nodes):
        seen = set()
        double = set()
        for node in nodes:
            if node.position in seen:
                double.add(node.position)
            else:
                seen.add(node.position)
        return set([i for i, node in enumerate(nodes) if node.position in double])

    def getNeighbors(self, current: MstarNode):
        neighbors = []
        options = []
        for i in range(self.n_agents):
            node: Node = current.nodes[i]
            options_i = []
            if node in current.collision_set:
                options_i.append(node)
                (x, y) = node.position
                moves = {0: (x - 1, y), 90: (x, y + 1), 180: (x + 1, y), 270: (x, y - 1)}
                options_i.append(Node(node.position, node, node.rotation + 90, node.h))
                options_i.append(Node(node.position, node, node.rotation - 90, node.h))
                options_i.append(
                    Node(moves[node.rotation], node, node.rotation, self.heuristic(i, moves[node.rotation])))
            else:
                nextPos = Astar(self.grid, node, self.goal.nodes[i]).solve()
                options_i.append(Node(nextPos[0], node, nextPos[1], self.heuristic(i, nextPos[0])))
            options.append(options_i)
        for element in itertools.product(*options):
            neighbors.append(list(element))
        return neighbors

    def heuristic(self, index, node):
        return abs(node[0] - self.goals[index].position[0]) + abs(node[1] - self.goals[index].position[1])


if __name__ == "__main__":
    problem_graph = [[0, 0, 0, 0, 0, 0, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 1, 0, 1, 0, 1, 0],
                     [0, 0, 0, 0, 0, 0, 0]]
    start = ((0, 0, 0), (0, 4, 0))
    end = ((0, 5, 90), (0, 0, 270))
    print(Mstar(problem_graph, start, end).solve())
