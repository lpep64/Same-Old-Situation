"""Hanoi problem utilities.

This module provides a simple recursive solver for the Tower of Hanoi.

Function:
  - solve_hanoi(n, source, target, auxiliary) -> List[Tuple[str, str]]

The solver returns a list of moves where each move is a 2-tuple
  (from_peg, to_peg)

Example:
  >>> solve_hanoi(2, 'A', 'C', 'B')
  [('A', 'B'), ('A', 'C'), ('B', 'C')]
"""
from typing import List, Tuple


def solve_hanoi(n: int, source: str = 'A', target: str = 'C', auxiliary: str = 'B') -> List[Tuple[str, str]]:
    """Return the list of moves to solve Tower of Hanoi recursively.

    Args:
        n: Number of disks (must be >= 0).
        source: Name/label of the source peg.
        target: Name/label of the target peg.
        auxiliary: Name/label of the auxiliary peg.

    Returns:
        A list of (from_peg, to_peg) tuples describing the moves in order.

    Raises:
        ValueError: If n is negative.
    """
    if not isinstance(n, int):
        raise TypeError("n must be an integer")
    if n < 0:
        raise ValueError("n must be >= 0")

    moves: List[Tuple[str, str]] = []

    def _recurse(k: int, a: str, b: str, c: str) -> None:
        # move k disks from a to b using c as auxiliary
        if k == 0:
            return
        _recurse(k - 1, a, c, b)
        moves.append((a, b))
        _recurse(k - 1, c, b, a)

    _recurse(n, source, target, auxiliary)
    return moves


if __name__ == '__main__':
    # Quick CLI demo / smoke test
    import sys

    try:
        n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    except ValueError:
        print("Usage: python hanoi_logic.py [n]")
        sys.exit(2)

    moves = solve_hanoi(n)
    print(f"Generated {len(moves)} moves for n={n} (expected {2**n - 1}):")
    for i, m in enumerate(moves, start=1):
        print(f"{i:3d}: {m[0]} -> {m[1]}")
