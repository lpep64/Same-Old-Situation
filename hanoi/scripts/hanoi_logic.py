from typing import List, Tuple

def solve_hanoi(n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
    """
    Generates the optimal sequence of moves for the Tower of Hanoi puzzle.
    """
    move_sequence = []
    _solve_recursive(move_sequence, n, source, destination, auxiliary)
    return move_sequence

def _solve_recursive(moves: List, n: int, source: str, dest: str, aux: str):
    """Helper recursive function."""
    if n == 1:
        moves.append((source, dest))
    else:
        _solve_recursive(moves, n - 1, source, aux, dest)
        moves.append((source, dest))
        _solve_recursive(moves, n - 1, aux, dest, source)

# # Example for verification:
# solution = solve_hanoi(3, 'A', 'C', 'B')
# print(f"Solution for 3 disks requires {len(solution)} moves: {solution}")