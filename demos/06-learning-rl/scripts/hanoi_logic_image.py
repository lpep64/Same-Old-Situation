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

def solve_hanoi(n: int = None, source: str = 'A', target: str = 'C', auxiliary: str = 'B', 
                initial_state: List[List[int]] = None) -> Tuple[List[Tuple[str, str]], Dict[str, Peg]]:
    """Return the list of moves to solve Tower of Hanoi recursively using state tracking.

    Args:
        n: Number of disks (must be >= 0). If initial_state is provided, n is ignored.
        source: Name/label of the source peg.
        target: Name/label of the target peg.
        auxiliary: Name/label of the auxiliary peg.
        initial_state: Optional initial configuration as list of three lists.
                      E.g., [[3,2,1],[],[]] for default start, [[3,2],[],[1]] after first move.
                      Disks are listed from bottom to top on each peg.

    Returns:
        A tuple containing:
        - A list of (from_peg, to_peg) tuples describing the moves in order.
        - A dictionary mapping peg names to Peg objects with final state.

    Raises:
        ValueError: If n is negative or initial_state is invalid.
        TypeError: If n is not an integer.
    """
    # Initialize pegs
    pegs: Dict[str, Peg] = {
        source: Peg(source),
        target: Peg(target),
        auxiliary: Peg(auxiliary)
    }
    
    peg_names = [source, auxiliary, target]  # Map indices to peg names
    
    if initial_state is not None:
        # Validate initial state
        if len(initial_state) != 3:
            raise ValueError("initial_state must contain exactly 3 lists (one per peg)")
        
        # Create all disks from initial state
        all_disk_sizes = []
        for peg_disks in initial_state:
            all_disk_sizes.extend(peg_disks)
        
        if not all_disk_sizes:
            n = 0
        else:
            n = max(all_disk_sizes)
            # Verify we have all disks from 1 to n
            if sorted(all_disk_sizes) != list(range(1, n + 1)):
                raise ValueError(f"initial_state must contain all disks from 1 to {n}")
        
        # Create disk objects
        disk_objects: Dict[int, Disk] = {}
        for size in range(1, n + 1):
            disk_objects[size] = Disk(size)
        
        # Place disks on pegs according to initial state
        for peg_idx, peg_disks in enumerate(initial_state):
            peg_name = peg_names[peg_idx]
            # Validate disks are in correct order (bottom to top, decreasing size)
            for i in range(len(peg_disks) - 1):
                if peg_disks[i] <= peg_disks[i + 1]:
                    raise ValueError(f"Invalid disk order on peg {peg_name}: larger disk must be below smaller disk")
            
            # Add disks to peg (bottom to top)
            for disk_size in peg_disks:
                pegs[peg_name].push(disk_objects[disk_size])
    else:
        # Default initialization: all disks on source peg
        if n is None:
            n = 3  # Default to 3 disks
        if not isinstance(n, int):
            raise TypeError("n must be an integer")
        if n < 0:
            raise ValueError("n must be >= 0")
        
        # Create disks and place them on source peg (largest to smallest)
        disks: List[Disk] = []
        for size in range(n, 0, -1):
            disk = Disk(size)
            disks.append(disk)
            pegs[source].push(disk)

    moves: List[Tuple[str, str, int, int, int]] = []  # (from, to, disk_size, from_height, to_height)

    def move_disk(from_peg: str, to_peg: str) -> None:
        """Move the top disk from one peg to another and record the move."""
        disk = pegs[from_peg].pop()
        from_height = len(pegs[from_peg])  # Height it was at before popping
        pegs[to_peg].push(disk)
        to_height = disk.height  # Height it's now at after pushing
        moves.append((from_peg, to_peg, disk.size, from_height, to_height))

    def get_auxiliary(peg1: str, peg2: str) -> str:
        """Get the third peg that's not peg1 or peg2."""
        all_pegs = {source, target, auxiliary}
        return list(all_pegs - {peg1, peg2})[0]

    def get_peg_for_disk(size: int) -> str:
        """Find which peg contains the specified disk."""
        for peg_name, peg in pegs.items():
            for disk in peg.disks:
                if disk.size == size:
                    return peg_name
        return None

    def can_place_disk(disk_size: int, peg_name: str) -> bool:
        """Check if a disk can be legally placed on a peg."""
        peg_obj = pegs[peg_name]
        if not peg_obj.disks:
            return True
        return peg_obj.disks[-1].size > disk_size
    
    def move_specific_disk(disk_size: int, dest_peg: str) -> None:
        """Move a specific disk to the destination peg recursively.
        
        Handles arbitrary initial states by recursively clearing obstacles.
        """
        current_peg = get_peg_for_disk(disk_size)
        
        # Already at destination
        if current_peg == dest_peg:
            return
        
        aux_peg = get_auxiliary(current_peg, dest_peg)
        
        # Get all disks on top of target disk (blocking it from leaving current peg)
        current_peg_obj = pegs[current_peg]
        blocking_disks = []
        found_target = False
        for disk in current_peg_obj.disks:
            if disk.size == disk_size:
                found_target = True
            elif found_target:
                blocking_disks.append(disk.size)
        
        # Move blocking disks out of the way
        # We need to move them somewhere they won't block our target disk's move
        for blocking_size in blocking_disks:
            # Try to find the best place for this blocking disk
            # Prefer moving to aux_peg, but if that's not possible, use the other peg
            if can_place_disk(blocking_size, aux_peg):
                move_specific_disk(blocking_size, aux_peg)
            else:
                # aux_peg is blocked, use dest_peg temporarily
                move_specific_disk(blocking_size, dest_peg)
        
        # Clear destination peg of any disks that would block this placement
        dest_peg_obj = pegs[dest_peg]
        blocking_at_dest = [d.size for d in dest_peg_obj.disks if d.size < disk_size]
        
        # Move blocking disks from destination
        for blocking_size in blocking_at_dest:
            # Move to the auxiliary peg
            move_specific_disk(blocking_size, aux_peg)
        
        # Now move the target disk
        current_peg = get_peg_for_disk(disk_size)  # Recheck location
        move_disk(current_peg, dest_peg)

    def solve_from_state() -> None:
        """Solve Tower of Hanoi from arbitrary initial state.
        
        Move each disk from largest to smallest to the target peg.
        """
        for disk_size in range(n, 0, -1):
            move_specific_disk(disk_size, target)

    solve_from_state()
    
    return moves, pegs


if __name__ == '__main__':
    import sys
    import json

    # Custom state for n=5
    initial_state = [[5], [4], [3, 2, 1]]
    
    print("Tower of Hanoi Solver - Custom State Demo")
    print("=" * 70)
    print(f"\nInitial state (n=5):")
    print(f"  Peg A: {initial_state[0]}")
    print(f"  Peg B: {initial_state[1]}")
    print(f"  Peg C: {initial_state[2]}")
    
    moves, pegs = solve_hanoi(initial_state=initial_state)
    
    print(f"\nGenerated {len(moves)} moves:\n")
    for i, move in enumerate(moves, start=1):
        from_peg, to_peg, disk_size, from_height, to_height = move
        print(f"{i:3d}: {from_peg} -> {to_peg}  |  Disk {disk_size} from height {from_height} to height {to_height}")
    
    print("\nFinal state:")
    for peg_name in ['A', 'B', 'C']:
        if peg_name in pegs:
            peg = pegs[peg_name]
            print(f"  {peg}")
