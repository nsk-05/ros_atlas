from typing import List, Set, Dict, Tuple
import heapq
from dataclasses import dataclass, field
from copy import deepcopy
import time

@dataclass(order=True)
class PrioritizedItem:
    priority: int
    item: object = field(compare=False)

@dataclass
class State:
    x: int
    y: int
    time: int = 0

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.time == other.time

    def __hash__(self):
        return hash((self.x, self.y, self.time))

@dataclass(order=True)
class Node:
    cost: int
    constraints: Dict[int, Set[Tuple[int, int, int]]] = field(default_factory=dict)
    solution: Dict[int, List[State]] = field(default_factory=dict)
    conflicts: List = field(default_factory=list)

class CBS:
    def __init__(self, grid: List[List[int]], starts: List[Tuple[int, int]], goals: List[Tuple[int, int]]):
        self.grid = grid
        self.starts = [State(x, y) for x, y in starts]
        self.goals = [State(x, y) for x, y in goals]
        self.movements = [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]  # Right, Down, Left, Up, Wait
        
    def is_valid(self, x: int, y: int) -> bool:
        return 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]) and self.grid[x][y] == 0

    def get_neighbors(self, state: State, constraints: Set[Tuple[int, int, int]]) -> List[State]:
        neighbors = []
        for dx, dy in self.movements:
            new_x, new_y = state.x + dx, state.y + dy
            if self.is_valid(new_x, new_y):
                new_state = State(new_x, new_y, state.time + 1)
                if (new_x, new_y, new_state.time) not in constraints:
                    neighbors.append(new_state)
        return neighbors

    def a_star(self, start: State, goal: State, constraints: Set[Tuple[int, int, int]]) -> List[State]:
        open_list = [PrioritizedItem(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}
        
        while open_list:
            current = heapq.heappop(open_list).item
            
            if current.x == goal.x and current.y == goal.y:
                path = self.reconstruct_path(came_from, current)
                return path
            
            for neighbor in self.get_neighbors(current, constraints):
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.manhattan_distance(neighbor, goal)
                    heapq.heappush(open_list, PrioritizedItem(f_score[neighbor], neighbor))
        
        return []

    def manhattan_distance(self, state1: State, state2: State) -> int:
        return abs(state1.x - state2.x) + abs(state1.y - state2.y)

    def reconstruct_path(self, came_from: Dict[State, State], current: State) -> List[State]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    def get_first_conflict(self, solution: Dict[int, List[State]]) -> Tuple[int, int, State]:
        max_time = max(len(path) for path in solution.values())
        
        for t in range(max_time):
            positions = {}
            for agent, path in solution.items():
                if t < len(path):
                    pos = path[t]
                    if pos in positions:
                        return (positions[pos], agent, pos)
                    positions[pos] = agent
                    
                    if t > 0 and t < len(path) and t-1 < len(solution[positions[pos]]):
                        # Check edge conflict
                        if path[t-1] == solution[positions[pos]][t] and path[t] == solution[positions[pos]][t-1]:
                            return (positions[pos], agent, pos)
                            
        return None

    def solve(self) -> Dict[int, List[State]]:
        root = Node(0)
        for i in range(len(self.starts)):
            path = self.a_star(self.starts[i], self.goals[i], set())
            if not path:
                return None
            root.solution[i] = path
            root.cost += len(path)

        open_list = [root]
        
        while open_list:
            P = heapq.heappop(open_list)
            
            conflict = self.get_first_conflict(P.solution)
            if not conflict:
                return P.solution
                
            a1, a2, conflict_state = conflict
            
            for agent in [a1, a2]:
                new_node = Node(0)
                new_node.constraints = deepcopy(P.constraints)
                
                if agent not in new_node.constraints:
                    new_node.constraints[agent] = set()
                new_node.constraints[agent].add((conflict_state.x, conflict_state.y, conflict_state.time))
                
                new_path = self.a_star(self.starts[agent], self.goals[agent], new_node.constraints[agent])
                if not new_path:
                    continue
                    
                new_node.solution = deepcopy(P.solution)
                new_node.solution[agent] = new_path
                new_node.cost = sum(len(path) for path in new_node.solution.values())
                
                heapq.heappush(open_list, new_node)
        
        return None

def visualize_grid(grid, starts, goals, paths=None):
    grid_copy = [row[:] for row in grid]
    
    # Mark starts with S and goals with G
    for i, (x, y) in enumerate(starts):
        grid_copy[x][y] = f'S{i}'
    for i, (x, y) in enumerate(goals):
        grid_copy[x][y] = f'G{i}'
    
    # If paths are provided, mark them on the grid
    if paths:
        for agent, path in paths.items():
            for state in path:
                if grid_copy[state.x][state.y] not in [f'S{agent}', f'G{agent}']:
                    grid_copy[state.x][state.y] = f'{agent}'
    
    # Print the grid
    for row in grid_copy:
        print(' '.join([str(cell).rjust(2) for cell in row]))

def main():
    grid = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    
    starts = [(1, 1), (17, 18)]  # Changed to (17, 18) to avoid wall
    goals = [(17, 18), (1, 1)]
    
    print("Initial grid with starts (S) and goals (G):")
    visualize_grid(grid, starts, goals)
    
    cbs = CBS(grid, starts, goals)
    start_time = time.time()
    solution = cbs.solve()
    end_time = time.time()
    
    if solution:
        print("\nSolution found in {:.2f} seconds!".format(end_time - start_time))
        for agent, path in solution.items():
            print(f"Agent {agent} path length: {len(path)}")
            print(f"Agent {agent} path: {[(state.x, state.y) for state in path]}")
        
        print("\nFinal grid with paths (agents marked as 0 and 1):")
        visualize_grid(grid, starts, goals, solution)
    else:
        print("No solution found")

if __name__ == "__main__":
    main()