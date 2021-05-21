from dataclasses import dataclass
from copy import copy
from typing import List, Tuple

DIR_FROM_ROT = {
    0: (0, -1),
    90: (1, 0),
    180: (0, 1),
    270: (-1, 0),
}


@dataclass
class State:
    x: int
    y: int
    r: int


def valid_state(state: State) -> bool:
    return (
        isinstance(state.x, int) and
        isinstance(state.y, int) and
        isinstance(state.r, int) and
        state.r in [0, 90, 180, 270]
    )


class Agent:
    def __init__(self, agent: State, goal: State):
        self.prev: State = copy(agent)
        self.state: State = agent
        self.goal: State = goal

    def has(self, state: State) -> bool:
        return self.state == state

    def in_wall(self, grid: List[List[int]]) -> bool:
        return grid[self.state.y][self.state.x] == 1

    def move(self, state: State) -> bool:
        print(f"{self.state} -> {state}")

        self.prev = self.state
        # Check if not moving.
        if self.state.x == state.x and self.state.y == state.y:
            # Check that rotation is not 180.
            # While not moving, agents can rotate 90 degrees in either direction or stay still,
            # So the only rotation they cannot have is 180 degrees from previous.
            is_valid = state.r != (state.r + 180) % 360
        else:
            # Can only move in direction specified by direction.
            (x, y) = DIR_FROM_ROT[self.state.r]
            is_valid = self.state.x + x == state.x and self.state.y + y == state.y
        # Update my state.
        self.state = state
        return is_valid

    def reached_goal(self) -> bool:
        return self.has(self.goal)


class Verifier:
    def __init__(self, grid: List[List[int]], agents: List[Tuple[int, int, int]], goals: List[Tuple[int, int, int]]):
        assert all(all(i in (0, 1) for i in row) for row in grid)
        self.grid = grid

        agents = [State(x, y, r) for (x, y, r) in agents]
        goals = [State(x, y, r) for (x, y, r) in goals]
        assert all(valid_state(state) for state in agents)
        assert all(valid_state(state) for state in goals)
        self.agents = [Agent(agent, goal)
                       for agent, goal in zip(agents, goals)]
        assert not any(agent.in_wall(self.grid) for agent in self.agents)
        # Check for collitions.
        for i, a in enumerate(self.agents):
            for b in self.agents[(i + 1):]:
                assert a.state != b.state

    def verify(self, solution: List[List["Node"]]):
        solution = [
            [State(node.position[0], node.position[1], node.rotation) for node in turn] for turn in solution
        ]
        assert all(all(valid_state(state) for state in turn)
                   for turn in solution)

        # Check that all moves are valid.
        for states in solution:
            for agent, state in zip(self.agents, states):
                assert agent.move(state)
                assert not agent.in_wall(self.grid)
            # Check for collitions.
            for i, a in enumerate(self.agents):
                for b in self.agents[(i + 1):]:
                    assert a.state != b.state
                    # Same edge collision.
                    if a.state == b.prev:
                        assert b.state != a.prev
                    elif b.state == a.prev:
                        assert b.prev != a.state

        # Check that agents have reached their goal.
        for agent in self.agents:
            assert agent.reached_goal()

        print("Verified.")
