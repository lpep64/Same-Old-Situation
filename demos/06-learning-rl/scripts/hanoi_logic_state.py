"""Hanoi problem utilities with state tracking.

This module provides a recursive solver for the Tower of Hanoi that
uses Disk and Peg classes to track the actual state of the puzzle.

Classes:
  - Disk: Represents a disk with size and position tracking
  - Peg: Represents a peg with a stack of disks

Function:
  - solve_hanoi(n, source, target, auxiliary) -> List[Tuple[str, str]]

The solver returns a list of moves where each move is a 2-tuple
  (from_peg, to_peg) and maintains the state using Disk and Peg objects.

Example:
  >>> solve_hanoi(2, 'A', 'C', 'B')
  [('A', 'B'), ('A', 'C'), ('B', 'C')]
"""
from typing import List, Tuple, Dict

class Disk:
    """Represents a disk in the Tower of Hanoi puzzle."""
    
    def __init__(self, size: int):
        self.size = size
        self.peg = None
        self.height = 0  # Height on the peg, 0 is bottom

    def __repr__(self):
        return f"Disk(size={self.size}, peg={self.peg.name if self.peg else None}, height={self.height})"


class Peg:
    """Represents a peg that holds a stack of disks."""
    
    def __init__(self, name: str):
        self.name = name
        self.disks: List[Disk] = []

    def push(self, disk: Disk):
        """Add a disk to the top of this peg."""
        if self.disks and self.disks[-1].size <= disk.size:
            raise ValueError(f"Cannot place larger disk (size {disk.size}) on top of smaller disk (size {self.disks[-1].size})")
        disk.peg = self
        disk.height = len(self.disks)
        self.disks.append(disk)

    def pop(self) -> Disk:
        """Remove and return the top disk from this peg."""
        if not self.disks:
            raise ValueError(f"No disks to pop from peg {self.name}")
        disk = self.disks.pop()
        # Update heights of remaining disks (though they don't change)
        disk.peg = None
        disk.height = 0
        return disk

    def peek(self) -> Disk:
        """Return the top disk without removing it."""
        if not self.disks:
            return None
        return self.disks[-1]

    def __repr__(self):
        return f"Peg({self.name}, disks={[d.size for d in self.disks]})"

    def __len__(self):
        return len(self.disks)

def solve_hanoi(n: int, source: str = 'A', target: str = 'C', auxiliary: str = 'B') -> Tuple[List[Tuple[str, str]], Dict[str, Peg]]:
    """Return the list of moves to solve Tower of Hanoi recursively using state tracking.

    Args:
        n: Number of disks (must be >= 0).
        source: Name/label of the source peg.
        target: Name/label of the target peg.
        auxiliary: Name/label of the auxiliary peg.

    Returns:
        A tuple containing:
        - A list of (from_peg, to_peg) tuples describing the moves in order.
        - A dictionary mapping peg names to Peg objects with final state.

    Raises:
        ValueError: If n is negative.
        TypeError: If n is not an integer.
    """
    if not isinstance(n, int):
        raise TypeError("n must be an integer")
    if n < 0:
        raise ValueError("n must be >= 0")

    # Initialize pegs
    pegs: Dict[str, Peg] = {
        source: Peg(source),
        target: Peg(target),
        auxiliary: Peg(auxiliary)
    }

    # Create disks and place them on source peg (largest to smallest)
    disks: List[Disk] = []
    for size in range(n, 0, -1):
        disk = Disk(size)
        disks.append(disk)
        pegs[source].push(disk)

    moves: List[Tuple[str, str, int, int, int]] = []  # (from, to, disk_size, from_height, to_height)

    def _recurse(k: int, a: str, b: str, c: str) -> None:
        """Move k disks from peg a to peg b using peg c as auxiliary."""
        if k == 0:
            return
        # Move k-1 disks from a to c using b
        _recurse(k - 1, a, c, b)
        
        # Move one disk from a to b
        disk = pegs[a].pop()
        from_height = len(pegs[a])  # Height it was at before popping
        pegs[b].push(disk)
        to_height = disk.height  # Height it's now at after pushing
        moves.append((a, b, disk.size, from_height, to_height))
        
        # Move k-1 disks from c to b using a
        _recurse(k - 1, c, b, a)

    _recurse(n, source, target, auxiliary)
    return moves, pegs


if __name__ == '__main__':
    # Quick CLI demo / smoke test
    import sys

    try:
        n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    except ValueError:
        print("Usage: python hanoi_logic_state.py [n]")
        sys.exit(2)

    print(f"Solving Tower of Hanoi for n={n} disks\n")
    
    moves, pegs = solve_hanoi(n)
    
    print(f"Generated {len(moves)} moves (expected {2**n - 1}):\n")
    for i, move in enumerate(moves, start=1):
        from_peg, to_peg, disk_size, from_height, to_height = move
        print(f"{i:3d}: {from_peg} -> {to_peg}  |  Disk {disk_size} from height {from_height} to height {to_height}")
    
    print("\nFinal state of pegs:")
    for peg_name in ['A', 'C', 'B']:
        if peg_name in pegs:
            peg = pegs[peg_name]
            print(f"  {peg}")
    
    # Verify all disks are on target peg
    target_peg = pegs['C']
    print(f"\nVerification: Target peg 'C' has {len(target_peg)} disks")
    if len(target_peg) == n:
        # Check if disks are in correct order (largest to smallest)
        sizes = [d.size for d in target_peg.disks]
        if sizes == list(range(n, 0, -1)):
            print("✓ Success! All disks are on target peg in correct order.")
        else:
            print(f"✗ Error: Disks are not in correct order: {sizes}")
    else:
        print(f"✗ Error: Expected {n} disks on target peg, found {len(target_peg)}")
