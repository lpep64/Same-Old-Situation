#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional
import random
import math

from niryo_ned_ros2_interfaces.srv import ToolCommand
from niryo_ned_ros2_interfaces.action import RobotMove
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from niryo_ned_ros2_interfaces.msg import ArmMoveCommand

# ============================================================================
# ABSTRACT BASE CLASS FOR SOLVING STRATEGIES
# ============================================================================

class HanoiSolver(ABC):
    """Abstract base class for Tower of Hanoi solving algorithms"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.move_sequence = []
    
    @abstractmethod
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        """
        Solve the Tower of Hanoi puzzle and return the move sequence
        
        Args:
            n: number of disks
            source: starting rod (e.g., 'A')
            destination: target rod (e.g., 'C')
            auxiliary: helper rod (e.g., 'B')
            
        Returns:
            List of tuples representing moves (from_rod, to_rod)
        """
        pass
    
    @abstractmethod
    def get_algorithm_name(self) -> str:
        """Return the name of the algorithm"""
        pass
    
    def log_info(self, message: str):
        """Helper method for logging"""
        if self.logger:
            self.logger.info(message)
        else:
            print(message)

# ============================================================================
# CONCRETE SOLVER IMPLEMENTATIONS
# ============================================================================

class RecursiveSolver(HanoiSolver):
    """Traditional recursive solution"""
    
    def get_algorithm_name(self) -> str:
        return "Recursive Algorithm"
    
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        self.move_sequence = []
        self._solve_recursive(n, source, destination, auxiliary)
        return self.move_sequence
    
    def _solve_recursive(self, n: int, source: str, destination: str, auxiliary: str):
        if n == 1:
            self.move_sequence.append((source, destination))
            self.log_info(f"Move disk from {source} to {destination}")
        else:
            # Move n-1 disks from source to auxiliary
            self._solve_recursive(n-1, source, auxiliary, destination)
            # Move bottom disk from source to destination
            self.move_sequence.append((source, destination))
            self.log_info(f"Move disk from {source} to {destination}")
            # Move n-1 disks from auxiliary to destination
            self._solve_recursive(n-1, auxiliary, destination, source)


class IterativeSolver(HanoiSolver):
    """Iterative solution using bit manipulation"""
    
    def get_algorithm_name(self) -> str:
        return "Iterative Algorithm"
    
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        self.move_sequence = []
        rods = [source, auxiliary, destination]
        
        # For even n, swap auxiliary and destination
        if n % 2 == 0:
            rods[1], rods[2] = rods[2], rods[1]
        
        total_moves = (2 ** n) - 1
        
        for i in range(total_moves):
            # Find which disk to move (trailing zeros + 1)
            disk = (i & -i).bit_length()
            
            # Calculate source and destination based on disk number and move count
            if disk % 2 == 1:  # Odd numbered disk
                from_rod = rods[i % 3]
                to_rod = rods[(i + 1) % 3]
            else:  # Even numbered disk
                from_rod = rods[(i + 1) % 3] if (i // (2 ** (disk - 1))) % 2 == 0 else rods[(i + 2) % 3]
                to_rod = rods[(i + 2) % 3] if (i // (2 ** (disk - 1))) % 2 == 0 else rods[(i + 1) % 3]
            
            self.move_sequence.append((from_rod, to_rod))
            self.log_info(f"Move {i+1}: {from_rod} to {to_rod}")
        
        return self.move_sequence


class SimulatedAnnealingSolver(HanoiSolver):
    """Simulated Annealing approach (educational/experimental)"""
    
    def get_algorithm_name(self) -> str:
        return "Simulated Annealing (Experimental)"
    
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        # This is a simplified educational example
        # In practice, SA isn't ideal for Hanoi since we know the optimal solution
        self.log_info("Using experimental simulated annealing approach...")
        
        # For demonstration, we'll use the recursive solution as base
        # but add some "exploration" concept
        recursive_solver = RecursiveSolver(self.logger)
        optimal_moves = recursive_solver.solve(n, source, destination, auxiliary)
        
        # Simulate some "optimization" process
        self.log_info("Exploring solution space...")
        time.sleep(0.5)  # Simulate computation time
        
        self.move_sequence = optimal_moves
        return self.move_sequence


class ReinforcementLearningSolver(HanoiSolver):
    """Mock RL solver (placeholder for actual RL implementation)"""
    
    def get_algorithm_name(self) -> str:
        return "Reinforcement Learning (Mock)"
    
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        self.log_info("Initializing RL agent...")
        self.log_info("Training on Tower of Hanoi environment...")
        
        # Mock training progress
        for episode in range(1, 6):
            self.log_info(f"Episode {episode}/5: Exploring action space...")
            time.sleep(0.2)
        
        self.log_info("Training complete. Deploying learned policy...")
        
        # For now, fall back to known optimal solution
        # In real implementation, this would use trained RL model
        recursive_solver = RecursiveSolver(self.logger)
        self.move_sequence = recursive_solver.solve(n, source, destination, auxiliary)
        
        return self.move_sequence


class DynamicProgrammingSolver(HanoiSolver):
    """Dynamic Programming approach using memoization"""
    
    def get_algorithm_name(self) -> str:
        return "Dynamic Programming"
    
    def __init__(self, logger=None):
        super().__init__(logger)
        self.memo = {}
    
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        self.move_sequence = []
        self.memo = {}
        self._solve_dp(n, source, destination, auxiliary)
        return self.move_sequence
    
    def _solve_dp(self, n: int, source: str, destination: str, auxiliary: str):
        # Create a unique key for memoization
        key = (n, source, destination, auxiliary)
        
        if key in self.memo:
            # Use cached result
            cached_moves = self.memo[key]
            self.move_sequence.extend(cached_moves)
            self.log_info(f"Using cached solution for {n} disks: {source}->{destination}")
            return cached_moves
        
        if n == 1:
            move = (source, destination)
            self.move_sequence.append(move)
            self.memo[key] = [move]
            self.log_info(f"Base case: Move disk from {source} to {destination}")
            return [move]
        
        # Build solution and cache it
        moves = []
        
        # Step 1: Move n-1 disks to auxiliary
        step1_moves = self._solve_dp(n-1, source, auxiliary, destination)
        moves.extend(step1_moves)
        
        # Step 2: Move bottom disk to destination
        bottom_move = (source, destination)
        self.move_sequence.append(bottom_move)
        moves.append(bottom_move)
        self.log_info(f"Move bottom disk from {source} to {destination}")
        
        # Step 3: Move n-1 disks from auxiliary to destination
        step3_moves = self._solve_dp(n-1, auxiliary, destination, source)
        moves.extend(step3_moves)
        
        self.memo[key] = moves
        return moves


# ============================================================================
# SOLVER FACTORY
# ============================================================================

class SolverFactory:
    """Factory class to create different solver instances"""
    
    @staticmethod
    def get_available_solvers() -> dict:
        """Return dictionary of available solvers"""
        return {
            'recursive': RecursiveSolver,
            'iterative': IterativeSolver,
            'simulated_annealing': SimulatedAnnealingSolver,
            'reinforcement_learning': ReinforcementLearningSolver,
            'dynamic_programming': DynamicProgrammingSolver,
        }
    
    @staticmethod
    def create_solver(solver_type: str, logger=None) -> HanoiSolver:
        """Create a solver instance of the specified type"""
        solvers = SolverFactory.get_available_solvers()
        
        if solver_type not in solvers:
            raise ValueError(f"Unknown solver type: {solver_type}. Available: {list(solvers.keys())}")
        
        return solvers[solver_type](logger)


# ============================================================================
# MODIFIED TOWER OF HANOI CONTROLLER
# ============================================================================

class TowerOfHanoiController(Node):
    # TODO 1: CONSTRUCTOR MODIFICATION
    # Currently this constructor only works for 3 disks. You need to:
    # 1. Add a 'num_disks' parameter to the constructor
    # 2. Store this parameter as an instance variable (self.num_disks)
    # 3. Use this parameter to initialize stack_counts and level_names dynamically
    
    def __init__(self, solver_type: str = 'recursive'):
        super().__init__('tower_of_hanoi_controller')

        # Create the specified solver
        self.solver = SolverFactory.create_solver(solver_type, self.get_logger())
        
        # Service clients for vacuum control
        self.vacuum_push_client = self.create_client(ToolCommand, '/robot2/niryo_robot/tools/push_air_vacuum_pump')
        self.vacuum_pull_client = self.create_client(ToolCommand, '/robot2/niryo_robot/tools/pull_air_vacuum_pump')

        # Action client for robot move
        self.robot_move_client = ActionClient(self, RobotMove, '/robot2/niryo_robot_arm_commander/robot_action')

        # Wait for services and actions
        self.wait_for_services()
        
        # Tower of Hanoi move sequence (will be populated by chosen algorithm)
        self.move_sequence = []
        
        # TODO 2: DYNAMIC STACK INITIALIZATION
        # Currently hardcoded for 3 disks. Replace this with dynamic initialization:
        # - Rod A should start with 'num_disks' disks
        # - Rods B and C should start empty (0 disks)
        self.stack_counts = {'A': 3, 'B': 0, 'C': 0}  # REPLACE THIS LINE
        
        # TODO 3: DYNAMIC LEVEL NAMES GENERATION
        # Currently hardcoded for 3 levels. You need to generate level names dynamically.
        # Suggestions:
        # - Use generic names like 'level_0', 'level_1', 'level_2', etc.
        # - Or create a method to generate appropriate level names
        # - Make sure each rod has exactly 'num_disks' level names
        self.level_names = {
            'A': ['low', 'middle', 'top'],
            'B': ['low', 'middle', 'top'],  
            'C': ['low', 'middle', 'high']
        }  # REPLACE THIS ENTIRE DICTIONARY

    def wait_for_services(self):
        self.get_logger().info('Waiting for services/actions...')
        self.vacuum_pull_client.wait_for_service()
        self.vacuum_push_client.wait_for_service()
        while not self.robot_move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for RobotMove action server...')
        self.get_logger().info('All services/actions are available.')

    # TODO 4: MODIFY SOLVE METHOD SIGNATURE
    # Add num_disks parameter to this method and use self.num_disks instead of hardcoded 3
    def solve_hanoi_with_algorithm(self, n: int, source: str, destination: str, auxiliary: str):
        """
        Solve Tower of Hanoi using the configured algorithm
        """
        self.get_logger().info(f"=== SOLVING WITH {self.solver.get_algorithm_name().upper()} ===")
        
        # Use the selected algorithm to generate move sequence
        self.move_sequence = self.solver.solve(n, source, destination, auxiliary)
        
        self.get_logger().info(f"Algorithm: {self.solver.get_algorithm_name()}")
        self.get_logger().info(f"Total moves required: {len(self.move_sequence)}")

    def activate_vacuum(self, mode: str = "neutral"):
        """Controls the vacuum system"""
        def send_vacuum_command(position, client):
            req = ToolCommand.Request()
            req.id = 32
            req.speed = 0
            req.hold_torque = 800
            req.max_torque = 1000
            req.position = position
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result()

        mode = mode.lower()
        if mode not in ["pull", "push", "neutral"]:
            self.get_logger().error(f"Invalid vacuum mode: {mode}")
            return

        if mode == "pull":
            position = 400
            client = self.vacuum_pull_client
        elif mode == "push":
            position = 2000
            client = self.vacuum_push_client
        else:  # neutral
            position = 1500
            client = self.vacuum_push_client

        self.get_logger().info(f"Setting vacuum to '{mode}' mode")
        result = send_vacuum_command(position, client)

        if result:
            self.get_logger().info(f"Vacuum '{mode}' command succeeded")
        else:
            self.get_logger().error(f"Vacuum '{mode}' command failed")

    def move_robot_joints(self, joint_values, description):
        """Move robot to specified joint configuration"""
        goal_msg = RobotMove.Goal()
        goal_msg.cmd.cmd_type = ArmMoveCommand.JOINTS
        goal_msg.cmd.joints = joint_values
        goal_msg.cmd.tcp_version = ArmMoveCommand.DH_CONVENTION

        self.get_logger().info(f'Moving to: {description}')
        future = self.robot_move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'Move to {description} was rejected!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.status != 0:
            self.get_logger().info(f'Move to {description} succeeded.')
        else:
            self.get_logger().error(f'Move to {description} failed: {result.message}')

    def get_pickup_position(self, rod, rod_positions):
        """Get pickup position for top disk of a rod"""
        stack_height = self.stack_counts[rod]
        if stack_height == 0:
            self.get_logger().error(f"Cannot pickup from empty rod {rod}")
            return None
            
        level_index = stack_height - 1
        level_name = self.level_names[rod][level_index]
        return rod_positions[rod][level_name]
    
    def get_drop_position(self, rod, rod_positions):
        """Get drop position for placing disk on a rod"""
        stack_height = self.stack_counts[rod]
        
        if stack_height == 0:
            level_name = self.level_names[rod][0]
            return rod_positions[rod][level_name]
        else:
            level_index = stack_height
            if level_index >= len(self.level_names[rod]):
                self.get_logger().error(f"Cannot drop on rod {rod} - stack too high")
                return None
            level_name = self.level_names[rod][level_index]
            return rod_positions[rod][level_name]

    def execute_hanoi_move(self, from_rod, to_rod, move_number, total_moves, rod_positions, home_joints, hold_pos):
        """Execute a single Tower of Hanoi move"""
        self.get_logger().info(f"=== EXECUTING MOVE {move_number}/{total_moves}: {from_rod} → {to_rod} ===")
        
        pickup_pos = self.get_pickup_position(from_rod, rod_positions)
        drop_pos = self.get_drop_position(to_rod, rod_positions)
        
        if pickup_pos is None or drop_pos is None:
            self.get_logger().error(f"Failed to get positions for move {from_rod} → {to_rod}")
            return
        
        # Execute movement sequence
        self.move_robot_joints(home_joints, "Home Position")
        time.sleep(0.3)
        
        self.activate_vacuum("neutral")
        time.sleep(0.6)
        
        camera_pos = rod_positions[from_rod]['camera']
        self.move_robot_joints(camera_pos, f"Camera Position for Rod {from_rod}")
        time.sleep(0.3)
        
        self.move_robot_joints(pickup_pos, f"Pickup from Rod {from_rod}")
        time.sleep(0.3)
        
        self.activate_vacuum("pull")
        time.sleep(0.6)
        
        self.move_robot_joints(hold_pos, "Hold Position")
        time.sleep(0.3)
        
        camera_pos = rod_positions[to_rod]['camera']
        self.move_robot_joints(camera_pos, f"Camera Position for Rod {to_rod}")
        time.sleep(0.3)
        
        self.move_robot_joints(drop_pos, f"Drop to Rod {to_rod}")
        time.sleep(0.3)
        
        self.activate_vacuum("push")
        time.sleep(0.6)
        
        # Update stack counts
        self.stack_counts[from_rod] -= 1
        self.stack_counts[to_rod] += 1
        
        self.get_logger().info(f"Move {move_number}/{total_moves} completed")


# ============================================================================
# USER INTERFACE FUNCTIONS
# ============================================================================

def display_available_algorithms():
    """Display available solving algorithms to user"""
    print("\n" + "="*60)
    print("TOWER OF HANOI - ALGORITHM SELECTION")
    print("="*60)
    
    solvers = SolverFactory.get_available_solvers()
    
    print("Available solving algorithms:")
    for i, (key, solver_class) in enumerate(solvers.items(), 1):
        # Create temporary instance to get algorithm name
        temp_solver = solver_class()
        print(f"{i}. {key} - {temp_solver.get_algorithm_name()}")
    
    print("\nEach algorithm will produce the same optimal solution")
    print("\nbut uses different computational approaches.")
    print("\nReinforcement Learning and Simulated Annealing are placeholder for now")
    return list(solvers.keys())

def get_user_algorithm_choice():
    """Get user's choice of algorithm"""
    available_algorithms = display_available_algorithms()
    
    while True:
        try:
            print(f"\nEnter choice (1-{len(available_algorithms)}) or algorithm name:")
            choice = input("> ").strip().lower()
            
            # Try to parse as number
            try:
                choice_num = int(choice)
                if 1 <= choice_num <= len(available_algorithms):
                    return available_algorithms[choice_num - 1]
            except ValueError:
                pass
            
            # Try to match algorithm name
            if choice in available_algorithms:
                return choice
            
            # Check partial matches
            matches = [alg for alg in available_algorithms if choice in alg]
            if len(matches) == 1:
                return matches[0]
            
            print(f"Invalid choice. Please enter 1-{len(available_algorithms)} or valid algorithm name.")
            
        except KeyboardInterrupt:
            print("\nExiting...")
            return None

# TODO 5: ADD USER INPUT FOR NUMBER OF DISKS
# Create a new function to get user input for number of disks (3, 4, or 5)
# Similar to get_user_algorithm_choice() but for disk count
# Include validation to ensure only 3, 4, or 5 disks are allowed
def get_user_disk_choice():
    """
    TODO: Implement this function
    Get user's choice for number of disks (3, 4, or 5)
    Should include:
    - Clear prompt asking for number of disks
    - Input validation (only accept 3, 4, or 5)
    - Error handling for invalid input
    - Return the chosen number as an integer
    """
    pass  # REPLACE WITH YOUR IMPLEMENTATION

def main(args=None):
    rclpy.init(args=args)
    
    # Get user's algorithm choice
    algorithm_choice = get_user_algorithm_choice()
    if algorithm_choice is None:
        return
    
    # TODO 6: GET USER'S DISK COUNT CHOICE
    # Add call to get_user_disk_choice() here
    # Store the result in a variable (e.g., num_disks)
    
    print(f"\nSelected algorithm: {algorithm_choice}")
    # TODO 7: PRINT SELECTED NUMBER OF DISKS
    # Add a print statement to show the selected number of disks
    
    print("Initializing robot controller...")
    
    # TODO 8: PASS num_disks TO CONTROLLER
    # Modify this line to pass the number of disks to the controller
    node = TowerOfHanoiController(algorithm_choice)

    # TODO 9: DEFINE POSITION CONFIGURATIONS FOR 4 AND 5 DISKS
    # The current rod_positions only has 3 levels per rod.
    # You need to add positions for 4th and 5th levels.
    # 
    # IMPORTANT NOTES FOR STUDENTS:
    # - Use Niryo Studio to obtain the joint positions (same method taught in class)
    # - You only need to define PICKUP positions - the code logically handles drop positions
    # - Physically stack disks to the desired heights and record joint positions using Niryo Studio
    # - The 'camera' position for each rod remains the same regardless of disk count
    # - Test each new position individually before running the full sequence
    # - Consider the physical constraints of your robot's workspace
    #
    # POSITION ACQUISITION PROCESS:
    # 1. Physically stack disks to create 4-disk and 5-disk configurations
    # 2. Use Niryo Studio to move the robot to pickup positions for each level
    # 3. Record the joint positions from Niryo Studio for each level
    # 4. Create separate position dictionaries for 3, 4, and 5 disks
    # 5. Use conditional logic to select appropriate dictionary based on num_disks
    #
    # NAMING CONVENTION SUGGESTION:
    # For 4 disks: 'level_0', 'level_1', 'level_2', 'level_3' (bottom to top)
    # For 5 disks: 'level_0', 'level_1', 'level_2', 'level_3', 'level_4' (bottom to top)
    # Keep 'camera' position unchanged for each rod
    
    # Current positions (only works for 3 disks)
    home_joints = [0.059, 0.274, -0.876, -0.072, -0.924, 0.005]
    camera_pos = [0.126, -0.173, -0.576, -0.362, -0.537, 0.146]
    hold_pos = [-0.046, 0.236, -0.493, -0.506, -0.778, 0.006]

    rod_positions = {
        'A': {
            'top':    [0.060, -0.290, -0.834, 0.052, -0.618, -1.140],
            'middle': [0.060, -0.329, -0.837, 0.046, -0.601, -1.141],
            'low':    [0.057, -0.388, -0.816, 0.118, -0.588, -1.141],
            'camera': [0.098, 0.149, -0.913, 0.065, -0.948, -1.141]
        },
        'B': {
            'low':    [0.949, -0.834, 0.025, -0.047, -0.825, -1.183],
            'middle': [0.938, -0.811, 0.033, 0.000, -0.868, -1.176],
            'top':    [0.938, -0.772, 0.010, -0.017, -0.851, -1.175],
            'camera': [0.961, -0.610, -0.080, -0.075, -0.890, -1.172]
        },
        'C': {
            'low':    [1.524, -0.438, -0.725, 0.040, -0.511, -1.167],
            'middle': [1.506, -0.410, -0.699, 0.155, -0.606, -1.367],
            'high':   [1.529, -0.367, -0.698, 0.074, -0.654, -1.617],
            'camera': [1.591, -0.105, -0.725, 0.054, -0.913, -1.621]
        }
    }
    
    # TODO 10: IMPLEMENT POSITION SELECTION LOGIC
    # Replace the above rod_positions with logic that selects the appropriate
    # position configuration based on num_disks
    # Example structure:
    # if num_disks == 3:
    #     rod_positions = positions_3_disks
    # elif num_disks == 4:
    #     rod_positions = positions_4_disks  # YOU NEED TO DEFINE THIS
    # elif num_disks == 5:
    #     rod_positions = positions_5_disks  # YOU NEED TO DEFINE THIS
    
    try:
        node.get_logger().info("=== TOWER OF HANOI SOLVER STARTING ===")
        node.get_logger().info(f"Using: {node.solver.get_algorithm_name()}")
        
        # TODO 11: USE DYNAMIC DISK COUNT IN SOLVE CALL
        # Replace the hardcoded '3' with the user-selected num_disks
        node.solve_hanoi_with_algorithm(3, 'A', 'C', 'B')  # MODIFY THIS LINE
        
        total_moves = len(node.move_sequence)
        node.get_logger().info(f"Solution found! Required moves: {total_moves}")
        
        # Print move sequence
        node.get_logger().info("=== MOVE SEQUENCE ===")
        for i, (from_rod, to_rod) in enumerate(node.move_sequence, 1):
            node.get_logger().info(f"Move {i}: {from_rod} → {to_rod}")
        
        # Execute moves
        input("\nPress Enter to start executing the solution...")
        
        for move_number, (from_rod, to_rod) in enumerate(node.move_sequence, 1):
            node.execute_hanoi_move(
                from_rod, to_rod, move_number, total_moves, 
                rod_positions, home_joints, hold_pos
            )
            
            if move_number < total_moves:
                time.sleep(0.5)
        
        node.get_logger().info("=== PUZZLE COMPLETED! ===")
        node.activate_vacuum("neutral")
        node.move_robot_joints(home_joints, "Final Home Position")
        
    except KeyboardInterrupt:
        node.get_logger().info('Operation interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'An error occurred: {str(e)}')
    finally:
        node.activate_vacuum("neutral")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# ============================================================================
# STUDENT ASSIGNMENT SUMMARY
# ============================================================================
"""
ASSIGNMENT: Modify this Tower of Hanoi solver to work with 4 and 5 disks

TASKS TO COMPLETE:
1. TODO 1: Modify constructor to accept num_disks parameter
2. TODO 2: Make stack initialization dynamic based on num_disks
3. TODO 3: Generate level names dynamically for any number of disks
4. TODO 4: Update solve method to use dynamic disk count
5. TODO 5: Implement get_user_disk_choice() function
6. TODO 6: Get user input for number of disks in main()
7. TODO 7: Display selected number of disks
8. TODO 8: Pass num_disks to controller constructor
9. TODO 9: Define new robot positions for 4th and 5th disk levels
10.TODO 10: Implement position selection logic based on num_disks
11.TODO 11: Use dynamic disk count in solve call

DELIVERABLES:
- Modified code that runs successfully for 3, 4, and 5 disks
- Documentation of new robot positions (how you determined them)
- Test results showing the solution works for all three disk counts
- Brief report on challenges encountered and solutions implemented

GRADING CRITERIA:
- Code compiles and runs without errors (30%)
- Correct algorithmic modifications (25%)
- Proper robot position definitions (25%)
- Code quality and documentation (10%)
- Testing and validation (10%)

BONUS POINTS:
- Add input validation and error handling
- Implement position interpolation for automatic position generation
- Add performance comparison between different algorithms for various disk counts
- Create a configuration file system for robot positions
"""
