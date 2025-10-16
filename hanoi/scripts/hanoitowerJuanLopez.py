#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional
from niryo_ned_ros2_interfaces.srv import ToolCommand
from niryo_ned_ros2_interfaces.action import RobotMove
from rclpy.action import ActionClient
from niryo_ned_ros2_interfaces.msg import ArmMoveCommand

# Constantes para tiempos de espera
SLEEP_SHORT = 0.3
SLEEP_MEDIUM = 0.5
SLEEP_LONG = 0.6

# ============================================================================
# ABSTRACT BASE CLASS FOR SOLVING STRATEGIES
# ============================================================================

class HanoiSolver(ABC):
    def __init__(self, logger=None):
        self.logger = logger
        self.move_sequence: List[Tuple[str, str]] = []

    @abstractmethod
    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        pass

    @abstractmethod
    def get_algorithm_name(self) -> str:
        pass

    def log_info(self, message: str):
        if self.logger:
            self.logger.info(message)
        else:
            print(message)

# ============================================================================
# CONCRETE SOLVER IMPLEMENTATIONS
# ============================================================================

class RecursiveSolver(HanoiSolver):
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
            self._solve_recursive(n-1, source, auxiliary, destination)
            self.move_sequence.append((source, destination))
            self.log_info(f"Move disk from {source} to {destination}")
            self._solve_recursive(n-1, auxiliary, destination, source)


class IterativeSolver(HanoiSolver):
    def get_algorithm_name(self) -> str:
        return "Iterative Algorithm"

    def solve(self, n: int, source: str, destination: str, auxiliary: str) -> List[Tuple[str, str]]:
        self.move_sequence = []
        stacks = {source: list(range(n, 0, -1)), auxiliary: [], destination: []}
        rods = [source, destination, auxiliary]

        if n % 2 == 0:
            rods[1], rods[2] = rods[2], rods[1]

        total_moves = (2 ** n) - 1
        for i in range(1, total_moves + 1):
            if i % 3 == 1:
                self._move_disk(stacks, rods[0], rods[1])
            elif i % 3 == 2:
                self._move_disk(stacks, rods[0], rods[2])
            else:
                self._move_disk(stacks, rods[1], rods[2])

        return self.move_sequence

    def _move_disk(self, stacks, rod1, rod2):
        if not stacks[rod1]:
            from_rod, to_rod = rod2, rod1
        elif not stacks[rod2]:
            from_rod, to_rod = rod1, rod2
        elif stacks[rod1][-1] < stacks[rod2][-1]:
            from_rod, to_rod = rod1, rod2
        else:
            from_rod, to_rod = rod2, rod1

        disk = stacks[from_rod].pop()
        stacks[to_rod].append(disk)
        self.move_sequence.append((from_rod, to_rod))
        self.log_info(f"Move disk {disk}: {from_rod} → {to_rod}")


class SolverFactory:
    @staticmethod
    def get_available_solvers() -> dict:
        return {
            'recursive': RecursiveSolver,
            'iterative': IterativeSolver,
        }

    @staticmethod
    def create_solver(solver_type: str, logger=None) -> HanoiSolver:
        solvers = SolverFactory.get_available_solvers()
        if solver_type not in solvers:
            raise ValueError(f"Unknown solver type: {solver_type}")
        return solvers[solver_type](logger)

# ============================================================================
# CONTROLLER
# ============================================================================

class TowerOfHanoiController(Node):
    def __init__(self, solver_type: str = 'recursive', num_disks: int = 3):
        super().__init__('tower_of_hanoi_controller')

        if num_disks not in (3, 4, 5):
            raise ValueError("num_disks must be 3, 4, or 5")

        self.num_disks = num_disks
        self.solver = SolverFactory.create_solver(solver_type, self.get_logger())

        self.vacuum_push_client = self.create_client(ToolCommand, '/robot2/niryo_robot/tools/push_air_vacuum_pump')
        self.vacuum_pull_client = self.create_client(ToolCommand, '/robot2/niryo_robot/tools/pull_air_vacuum_pump')
        self.robot_move_client = ActionClient(self, RobotMove, '/robot2/niryo_robot_arm_commander/robot_action')

        self.wait_for_services()

        self.stack_counts = {'A': self.num_disks, 'B': 0, 'C': 0}
        self.level_names = {rod: [f'level_{i}' for i in range(num_disks)] for rod in ('A','B','C')}
        self.move_sequence: List[Tuple[str, str]] = []

    def wait_for_services(self):
        self.get_logger().info('Waiting for services/actions...')
        try:
            self.vacuum_pull_client.wait_for_service()
            self.vacuum_push_client.wait_for_service()
        except Exception as e:
            self.get_logger().error(f"Error waiting for vacuum services: {e}")
            raise
        while not self.robot_move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for RobotMove action server...')
        self.get_logger().info('All services/actions are available.')

    def solve_hanoi_with_algorithm(self, source: str, destination: str, auxiliary: str):
        # Reinicia contadores por seguridad si se re-ejecuta
        # FIX: garantizar estado consistente antes de resolver
        self.stack_counts = {'A': self.num_disks, 'B': 0, 'C': 0}
        self.get_logger().info(f"=== SOLVING WITH {self.solver.get_algorithm_name().upper()} ===")
        self.move_sequence = self.solver.solve(self.num_disks, source, destination, auxiliary)
        self.get_logger().info(f"Algorithm: {self.solver.get_algorithm_name()}")
        self.get_logger().info(f"Total moves required: {len(self.move_sequence)}")

    def activate_vacuum(self, mode: str = "neutral") -> bool:
        def send_vacuum_command(position: int, client) -> bool:
            req = ToolCommand.Request()
            req.id = 32
            req.speed = 0
            req.hold_torque = 800
            req.max_torque = 1000
            req.position = position

            try:
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                result = future.result()
                # FIX: comprobar result no None
                if result is None:
                    self.get_logger().error("Vacuum service returned no result")
                    return False
                return True
            except Exception as e:
                self.get_logger().error(f"Vacuum command exception: {e}")
                return False

        mode = mode.lower()
        if mode not in ["pull", "push", "neutral"]:
            self.get_logger().error(f"Invalid vacuum mode: {mode}")
            return False

        if mode == "pull":
            position = 400
            client = self.vacuum_pull_client
        elif mode == "push":
            position = 2000
            client = self.vacuum_push_client
        else:
            position = 1500
            client = self.vacuum_push_client

        self.get_logger().info(f"Setting vacuum to '{mode}' mode")
        success = send_vacuum_command(position, client)

        if success:
            self.get_logger().info(f"Vacuum '{mode}' command succeeded")
        else:
            self.get_logger().error(f"Vacuum '{mode}' command failed")

        return success

    def move_robot_joints(self, joint_values: List[float], description: str) -> bool:
        goal_msg = RobotMove.Goal()
        goal_msg.cmd.cmd_type = ArmMoveCommand.JOINTS
        goal_msg.cmd.joints = joint_values
        goal_msg.cmd.tcp_version = ArmMoveCommand.DH_CONVENTION

        self.get_logger().info(f'Moving to: {description}')

        try:
            future = self.robot_move_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, future)
            goal_handle = future.result()
            # FIX: comprobar goal_handle válido
            if goal_handle is None:
                self.get_logger().error(f'No goal handle returned for {description}')
                return False

            if not goal_handle.accepted:
                self.get_logger().error(f'Move to {description} was rejected!')
                return False

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result_wrapper = result_future.result()
            if result_wrapper is None or result_wrapper.result is None:
                self.get_logger().error(f"No result received for move to {description}")
                return False

                # Aquí se comprueba el status del wrapper, no del result
            if result_wrapper.status != 0:  # STATUS_SUCCEEDED
                self.get_logger().info(f"Move to {description} succeeded.")
                return True
            else:
                msg = getattr(result_wrapper.result, "message", "unknown error")
                self.get_logger().error(f"Move to {description} failed with status {result_wrapper.status}: {msg}")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception during move to {description}: {e}")
            return False

    def get_pickup_position(self, rod: str, rod_positions: dict) -> Optional[List[float]]:
        stack_height = self.stack_counts[rod]
        if stack_height <= 0:
            self.get_logger().error(f"Cannot pickup from empty rod {rod}")
            return None
        level_index = stack_height - 1
        # FIX: comprobación de rango robusta
        if level_index < 0 or level_index >= len(self.level_names[rod]):
            self.get_logger().error(f"Pickup level index out of range for rod {rod}: {level_index}")
            return None
        level_name = self.level_names[rod][level_index]
        return rod_positions[rod][level_name]

    def get_drop_position(self, rod: str, rod_positions: dict) -> Optional[List[float]]:
        stack_height = self.stack_counts[rod]
        level_index = stack_height  # siguiente nivel libre
        # FIX: proteger contra desbordamiento de niveles
        if level_index < 0 or level_index >= len(self.level_names[rod]):
            self.get_logger().error(f"Cannot drop on rod {rod} - stack full (index {level_index})")
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


    #def execute_hanoi_move(self, from_rod: str, to_rod: str, move_number: int, total_moves: int,
    #                        rod_positions: dict, home_joints: List[float], hold_pos: List[float]):
    #     self.get_logger().info(f"=== EXECUTING MOVE {move_number}/{total_moves}: {from_rod} → {to_rod} ===")

    #     # FIX: validar estado antes de calcular posiciones
    #     if self.stack_counts[from_rod] <= 0:
    #         raise RuntimeError(f"Invalid move: source rod {from_rod} is empty")

    #     pickup_pos = self.get_pickup_position(from_rod, rod_positions)
    #     drop_pos = self.get_drop_position(to_rod, rod_positions)

    #     if pickup_pos is None or drop_pos is None:
    #         raise RuntimeError(f"Invalid positions for move {from_rod} → {to_rod}")

    #     # Ir a home
    #     if not self.move_robot_joints(home_joints, "Home Position"):
    #         raise RuntimeError("Failed to move to home position")
    #     time.sleep(SLEEP_SHORT)    # def execute_hanoi_move(self, from_rod: str, to_rod: str, move_number: int, total_moves: int,
    #                        rod_positions: dict, home_joints: List[float], hold_pos: List[float]):
    #     self.get_logger().info(f"=== EXECUTING MOVE {move_number}/{total_moves}: {from_rod} → {to_rod} ===")

    #     # FIX: validar estado antes de calcular posiciones
    #     if self.stack_counts[from_rod] <= 0:
    #         raise RuntimeError(f"Invalid move: source rod {from_rod} is empty")

    #     pickup_pos = self.get_pickup_position(from_rod, rod_positions)
    #     drop_pos = self.get_drop_position(to_rod, rod_positions)

    #     if pickup_pos is None or drop_pos is None:
    #         raise RuntimeError(f"Invalid positions for move {from_rod} → {to_rod}")

    #     # Ir a home
    #     if not self.move_robot_joints(home_joints, "Home Position"):
    #         raise RuntimeError("Failed to move to home position")
    #     time.sleep(SLEEP_SHORT)

    #     # Vacío neutral
    #     if not self.activate_vacuum("neutral"):
    #         raise RuntimeError("Failed to set vacuum to neutral")
    #     time.sleep(SLEEP_LONG)

    #     # Posición de cámara origen
    #     cam_from = rod_positions[from_rod].get('camera')
    #     if cam_from and not self.move_robot_joints(cam_from, f"Camera Position for Rod {from_rod}"):
    #         raise RuntimeError(f"Failed to move to camera position for rod {from_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Pickup
    #     if not self.move_robot_joints(pickup_pos, f"Pickup from Rod {from_rod}"):
    #         raise RuntimeError(f"Failed to pickup from rod {from_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Activar vacío
    #     if not self.activate_vacuum("pull"):
    #         raise RuntimeError("Failed to activate vacuum pull")
    #     time.sleep(SLEEP_LONG)

    #     # Hold position
    #     if not self.move_robot_joints(hold_pos, "Hold Position"):
    #         raise RuntimeError("Failed to move to hold position")
    #     time.sleep(SLEEP_SHORT)

    #     # Posición de cámara destino
    #     cam_to = rod_positions[to_rod].get('camera')
    #     if cam_to and not self.move_robot_joints(cam_to, f"Camera Position for Rod {to_rod}"):
    #         raise RuntimeError(f"Failed to move to camera position for rod {to_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Drop
    #     if not self.move_robot_joints(drop_pos, f"Drop to Rod {to_rod}"):
    #         raise RuntimeError(f"Failed to drop to rod {to_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Soltar vacío
    #     if not self.activate_vacuum("push"):
    #         raise RuntimeError("Failed to release vacuum")
    #     time.sleep(SLEEP_LONG)

    #     # Actualizar contadores (después de confirmar drop)
    #     self.stack_counts[from_rod] -= 1
    #     self.stack_counts[to_rod] += 1

    #     self.get_logger().info(f"Move {move_number}/{total_moves} completed")

    #     # Vacío neutral
    #     if not self.activate_vacuum("neutral"):
    #         raise RuntimeError("Failed to set vacuum to neutral")
    #     time.sleep(SLEEP_LONG)

    #     # Posición de cámara origen
    #     cam_from = rod_positions[from_rod].get('camera')
    #     if cam_from and not self.move_robot_joints(cam_from, f"Camera Position for Rod {from_rod}"):
    #         raise RuntimeError(f"Failed to move to camera position for rod {from_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Pickup
    #     if not self.move_robot_joints(pickup_pos, f"Pickup from Rod {from_rod}"):
    #         raise RuntimeError(f"Failed to pickup from rod {from_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Activar vacío
    #     if not self.activate_vacuum("pull"):
    #         raise RuntimeError("Failed to activate vacuum pull")
    #     time.sleep(SLEEP_LONG)

    #     # Hold position
    #     if not self.move_robot_joints(hold_pos, "Hold Position"):
    #         raise RuntimeError("Failed to move to hold position")
    #     time.sleep(SLEEP_SHORT)

    #     # Posición de cámara destino
    #     cam_to = rod_positions[to_rod].get('camera')
    #     if cam_to and not self.move_robot_joints(cam_to, f"Camera Position for Rod {to_rod}"):
    #         raise RuntimeError(f"Failed to move to camera position for rod {to_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Drop
    #     if not self.move_robot_joints(drop_pos, f"Drop to Rod {to_rod}"):
    #         raise RuntimeError(f"Failed to drop to rod {to_rod}")
    #     time.sleep(SLEEP_SHORT)

    #     # Soltar vacío
    #     if not self.activate_vacuum("push"):
    #         raise RuntimeError("Failed to release vacuum")
    #     time.sleep(SLEEP_LONG)

    #     # Actualizar contadores (después de confirmar drop)
    #     self.stack_counts[from_rod] -= 1
    #     self.stack_counts[to_rod] += 1

        # self.get_logger().info(f"Move {move_number}/{total_moves} completed")

# ============================================================================
# USER INPUT FUNCTIONS
# ============================================================================

def display_available_algorithms():
    solvers = SolverFactory.get_available_solvers()
    print("\nAvailable algorithms:")
    for i, (key, solver_class) in enumerate(solvers.items(), 1):
        temp_solver = solver_class()
        print(f"{i}. {key} - {temp_solver.get_algorithm_name()}")
    return list(solvers.keys())

def get_user_algorithm_choice():
    available_algorithms = display_available_algorithms()
    while True:
        choice = input("Choose algorithm (number or name): ").strip().lower()
        try:
            num = int(choice)
            if 1 <= num <= len(available_algorithms):
                return available_algorithms[num-1]
        except ValueError:
            if choice in available_algorithms:
                return choice
        print("Invalid choice, try again.")

def get_user_disk_choice():
    while True:
        try:
            choice = int(input("Enter number of disks (3, 4, or 5): "))
            if choice in [3, 4, 5]:
                return choice
            else:
                print("Only 3, 4, or 5 are allowed.")
        except ValueError:
            print("Invalid input, enter a number.")

# ============================================================================
# MAIN
# ============================================================================

def main(args=None):
    rclpy.init(args=args)

    algorithm_choice = get_user_algorithm_choice()
    num_disks = get_user_disk_choice()

    print(f"\nSelected algorithm: {algorithm_choice}")
    print(f"Selected number of disks: {num_disks}")

    home_joints = [0.059, 0.274, -0.876, -0.072, -0.925, 0.006]
    hold_pos   = [-0.046, 0.236, -0.493, -0.506, -0.778, 0.006]

    node = TowerOfHanoiController(algorithm_choice, num_disks)

    # Definir posiciones (calibradas para 3 discos)
    positions_3_disks = {
        'A': {
            'level_0': [-0.016, -0.37, -0.813, -0.008, -0.44, 0.032],
            'level_1': [-0.017, -0.334, -0.795, 0, -0.525, 0.032],
            'level_2': [-0.017, -0.275, -0.816, 0.051, -0.537, 0.032],
            'camera':  [-0.10,  0.254, -0.752, 0.026, -0.960, -0.084],
        },
        'B': {
            'level_0': [0.459, -0.458, -0.649, 0.336, -0.528, 0.118],
            'level_1': [0.479, -0.397, -0.681, 0.299, -0.473, -1.118],
            'level_2': [0.467, -0.340, -0.720, 0.284, -0.446, 0.120],
            'camera':  [0.453, 0.003, -0.510, 0.094, -1.253, 0.144],
        },
        'C': {
            'level_0': [0.946, -0.888, 0.045, -0.209, -0.767, 0.195],
            'level_1': [0.957, -0.834, -0.011, -0.207, -0.732, 0.204],
            'level_2': [0.957, -0.791, -0.043, -0.141, -0.704, 0.207],
            'camera':  [0.928, -0.469, 0.105, 0.092, -1.092, 0.161],
        },
    }

    # IMPORTANTE: Mide estas posiciones con Niryo Studio antes de usar 4 o 5 discos
    positions_4_disks = {
        'A': {
            'level_0': [-0.016, -0.37, -0.813, -0.008, -0.44, 0.032],
            'level_1': [-0.017, -0.334, -0.795, 0, -0.525, 0.032],
            'level_2': [-0.017, -0.275, -0.816, 0.051, -0.537, 0.032],
            'level_3': [-0.03, -0.235, -0.808, 0.002, -0.575, 0.02],
            'camera':  [-0.10,  0.254, -0.752, 0.026, -0.960, -0.084],
        },
        'B': {
            'level_0': [0.459, -0.458, -0.649, 0.336, -0.528, 0.118],
            'level_1': [0.479, -0.397, -0.681, 0.299, -0.473, -1.118],
            'level_2': [0.467, -0.340, -0.720, 0.284, -0.446, 0.120],
            'level_3': [0.465, -0.294, -0.736, 0.275, -0.385, 0.121],
            'camera':  [0.453, 0.003, -0.510, 0.094, -1.253, 0.144],
        },
        'C': {
            'level_0': [0.946, -0.888, 0.045, -0.209, -0.767, 0.195],
            'level_1': [0.957, -0.834, -0.011, -0.207, -0.732, 0.204],
            'level_2': [0.957, -0.791, -0.043, -0.141, -0.704, 0.207],
            'level_3': [0.94, -0.744, -0.108, -0.167, -0.37, 0.215],
            'camera':  [0.928, -0.469, 0.105, 0.092, -1.092, 0.161],
        },
    }
    positions_5_disks = {
         'A': {
            'level_0': [-0.016, -0.37, -0.813, -0.008, -0.44, 0.032],
            'level_1': [-0.017, -0.334, -0.795, 0, -0.525, 0.032],
            'level_2': [-0.017, -0.275, -0.816, 0.051, -0.537, 0.032],
            'level_3': [-0.03, -0.235, -0.808, 0.002, -0.575, 0.02],
            'level_4': [-0.007, -0.175, -0.823, 0.006, -0.578, 0.019],
            'camera':  [-0.10,  0.254, -0.752, 0.026, -0.960, -0.084],
        },
        'B': {
            'level_0': [0.544, -0.49, -0.596, -0.09, -0.666, 0.19],
            'level_1': [0.544, -0.441, -0.608, -0.11, -0.695, 0.19],
            'level_2': [0.561, -0.397, -0.614, -0.113, -0.7, 0.19],
            'level_3': [0.561, -0.344, -0.637, -0.08, -0.7, 0.19],
            'level_4': [0.533, -0.311, -0.625, 0.005, -0.701, 0.19],
            'camera':  [0.453, 0.003, -0.510, 0.094, -1.253, 0.144],
        },
        'C': {
            'level_0': [0.946, -0.888, 0.045, -0.209, -0.767, 0.195],
            'level_1': [0.957, -0.834, -0.011, -0.207, -0.732, 0.204],
            'level_2': [0.957, -0.791, -0.043, -0.141, -0.704, 0.207],
            'level_3': [0.94, -0.744, -0.108, -0.167, -0.37, 0.215],
            'level_4': [0.975, -0.747, -0.024, -0.238, -0.715, 0.227],
            'camera':  [0.928, -0.469, 0.105, 0.092, -1.092, 0.161],
        },
    }

    if num_disks == 3:
        rod_positions = positions_3_disks
    elif num_disks == 4:
        if not positions_4_disks:
            print("ERROR: 4-disk positions not calibrated yet!")
            print("Please calibrate positions using Niryo Studio and update positions_4_disks dictionary.")
            node.destroy_node()
            rclpy.shutdown()
            return
        rod_positions = positions_4_disks
    else:
        if not positions_5_disks:
            print("ERROR: 5-disk positions not calibrated yet!")
            print("Please calibrate positions using Niryo Studio and update positions_5_disks dictionary.")
            node.destroy_node()
            rclpy.shutdown()
            return
        rod_positions = positions_5_disks



    try:
        node.solve_hanoi_with_algorithm('A', 'C', 'B')
        total_moves = len(node.move_sequence)
        print(f"Solution requires {total_moves} moves")

        input("Press Enter to start executing...")

        for move_number, (from_rod, to_rod) in enumerate(node.move_sequence, 1):
            node.execute_hanoi_move(from_rod, to_rod, move_number, total_moves, rod_positions, home_joints, hold_pos)
            if move_number < total_moves:
                time.sleep(SLEEP_MEDIUM)

        node.activate_vacuum("neutral")
        node.move_robot_joints(home_joints, "Final Home Position")
        node.get_logger().info("=== PUZZLE COMPLETED! ===")

    except RuntimeError as e:
        node.get_logger().error(f"Execution failed: {e}")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        try:
            node.activate_vacuum("neutral")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
