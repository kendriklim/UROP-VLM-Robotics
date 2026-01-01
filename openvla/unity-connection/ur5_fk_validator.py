"""
UR5 Forward and Inverse Kinematics Validator

This script validates both FK and IK for the UR5 robot:
- FK Validation: Connects to Unity TCP server and validates forward kinematics
- IK Validation: Tests inverse kinematics solutions and round-trip accuracy
- IK Server Testing: Tests the Python IK server if running

Binary protocol is much faster than JSON (no parsing overhead, smaller data size).
"""

import socket
import struct
import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3, UnitQuaternion
import os
import time


class UR5FKValidator:
    """UR5 Forward Kinematics Validator using roboticstoolbox"""

    def __init__(
            self,
            host='localhost',
            port=5005,
            tolerance=0.01,
            coordinate_mode='auto'):
        """
        Initialize the validator.

        Args:
            host (str): TCP server host
            port (int): TCP server port
            tolerance (float): Maximum allowable position error in meters
            coordinate_mode (str): Coordinate transformation mode:
                - 'none': No transformation
                - 'unity_to_ros': Unity (Y-up) to ROS (Z-up)
                - 'auto': Try multiple transformations and report (default)
        """
        self.host = host
        self.port = port
        self.tolerance = tolerance
        self.coordinate_mode = coordinate_mode
        self.socket = None

        # Load UR5 model from URDF file
        urdf_path = os.path.join(os.path.dirname(__file__), 'ur5.urdf')
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")

        self.ur5 = rtb.Robot.URDF(urdf_path)
        print(f"Loaded UR5 robot from URDF: {urdf_path}")
        print(self.ur5)

        print(rtb.models.UR5())

    def transform_coordinates(self, unity_pos, mode):
        """
        Transform coordinates between Unity and robotics conventions.

        Unity:     X-right, Y-up, Z-forward (left-handed)
        Standard robotics (ROS/RTB): X-forward, Y-left, Z-up (right-handed)

        Args:
            unity_pos: Position in Unity coordinates [x, y, z]
            mode: Transformation mode string

        Returns:
            Transformed position
        """
        if mode == 'none':
            return unity_pos
        elif mode == 'unity_to_ros':
            # Unity to ROS: [X_ros, Y_ros, Z_ros] = [Z_unity, -X_unity,
            # Y_unity]
            return np.array([unity_pos[2], -unity_pos[0], unity_pos[1]])
        else:
            return unity_pos

    def connect(self):
        """Establish TCP connection to the server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect(self):
        """Close TCP connection"""
        if self.socket:
            self.socket.close()
            print("Disconnected from server")

    def receive_data(self):
        """
        Receive data from TCP server in binary format.

        Binary format (72 bytes total):
            9 doubles: [ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6]
            All values are float64 (double precision, little-endian)

        Returns:
            tuple: (end_effector_position, joint_angles) as numpy arrays, or (None, None) if error
        """
        try:
            # Binary protocol: 9 doubles (72 bytes)
            # Format: ee_x, ee_y, ee_z, j1, j2, j3, j4, j5, j6
            data = self.socket.recv(72)
            if len(data) != 72:
                print(f"Expected 72 bytes, got {len(data)}")
                return None, None

            # Unpack 9 doubles (little-endian)
            values = struct.unpack('<9d', data)

            end_effector = np.array(values[:3])
            joint_angles = np.array(values[3:])

            return end_effector, joint_angles

        except struct.error as e:
            print(f"Binary unpacking error: {e}")
            return None, None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None, None

    def calculate_forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics for given joint angles.

        Args:
            joint_angles (np.ndarray): Array of 6 joint angles in radians

        Returns:
            np.ndarray: End effector position [x, y, z]
        """
        # Calculate forward kinematics using built-in UR5 model
        T = self.ur5.fkine(joint_angles)

        # Extract position (translation vector)
        position = T.t

        return position

    def calculate_inverse_kinematics(self, target_position, target_rotation=None, q0=None):
        """
        Calculate inverse kinematics for target pose.

        Args:
            target_position (np.ndarray): Target position [x, y, z]
            target_rotation (np.ndarray or UnitQuaternion, optional): Target rotation
            q0 (np.ndarray, optional): Initial guess for joint angles

        Returns:
            tuple: (success: bool, solution: np.ndarray or None, iterations: int)
        """
        try:
            # Create SE3 transform
            if target_rotation is None:
                # Default orientation - pointing down
                T_target = SE3.Trans(target_position)
            elif isinstance(target_rotation, UnitQuaternion):
                T_target = SE3.Rt(target_rotation.R, target_position)
            else:
                # Assume it's a rotation matrix
                T_target = SE3.Rt(target_rotation, target_position)

            # Use initial guess if provided, otherwise use zeros
            if q0 is None:
                q0 = np.zeros(6)

            # Solve IK using Levenberg-Marquardt
            result = self.ur5.ik_LM(T_target, q0=q0, tol=1e-6)

            # Handle different return types from roboticstoolbox versions
            if isinstance(result, tuple):
                # Older version returns (q, success, error, iterations, ...)
                # or (q, success)
                if len(result) >= 2:
                    q_result = result[0]
                    success = result[1] == 0 if isinstance(result[1], int) else bool(result[1])
                    iterations = result[3] if len(result) > 3 else 0
                    return success, q_result if success else None, iterations
                else:
                    return False, None, 0
            else:
                # Newer version returns object with attributes
                return result.success, result.q if result.success else None, result.iterations

        except Exception as e:
            print(f"IK calculation error: {e}")
            import traceback
            traceback.print_exc()
            return False, None, 0

    def test_ik_round_trip(self, joint_angles, position_tolerance=0.001, angle_tolerance=0.01):
        """
        Test IK by doing a round trip: joints -> FK -> IK -> FK

        Args:
            joint_angles (np.ndarray): Starting joint angles
            position_tolerance (float): Position error tolerance in meters
            angle_tolerance (float): Joint angle error tolerance in radians

        Returns:
            dict: Test results with success status and errors
        """
        # Step 1: Calculate FK from original joints
        T_original = self.ur5.fkine(joint_angles)
        pos_original = T_original.t
        rot_original = UnitQuaternion(T_original.R)

        # Step 2: Solve IK to get back to that pose
        success, ik_solution, iterations = self.calculate_inverse_kinematics(
            pos_original, rot_original, q0=joint_angles
        )

        if not success:
            return {
                'success': False,
                'error': 'IK solution failed',
                'iterations': iterations
            }

        # Step 3: Calculate FK from IK solution
        T_reconstructed = self.ur5.fkine(ik_solution)
        pos_reconstructed = T_reconstructed.t

        # Step 4: Compare positions
        position_error = np.linalg.norm(pos_reconstructed - pos_original)
        angle_error = np.linalg.norm(ik_solution - joint_angles)

        position_valid = position_error <= position_tolerance
        angle_valid = angle_error <= angle_tolerance

        return {
            'success': position_valid,
            'position_error': position_error,
            'angle_error': angle_error,
            'position_valid': position_valid,
            'angle_valid': angle_valid,
            'original_joints': joint_angles,
            'ik_solution': ik_solution,
            'original_position': pos_original,
            'reconstructed_position': pos_reconstructed,
            'iterations': iterations
        }

    def validate(self, reported_position, calculated_position):
        """
        Validate that reported and calculated positions match within tolerance.

        Args:
            reported_position (np.ndarray): Reported end effector position
            calculated_position (np.ndarray): Calculated end effector position

        Returns:
            tuple: (bool: is_valid, float: error_magnitude, np.ndarray: error_vector)
        """
        # Calculate error
        error = calculated_position - reported_position
        error_magnitude = np.linalg.norm(error)

        # Check if within tolerance
        is_valid = error_magnitude <= self.tolerance

        return is_valid, error_magnitude, error

    def run_continuous_validation(self):
        """Continuously receive data and validate forward kinematics"""
        if not self.connect():
            return

        print(
            f"\nValidating forward kinematics (BINARY protocol, tolerance: {self.tolerance}m)")
        print(f"Coordinate mode: {self.coordinate_mode}")
        print("-" * 70)

        try:
            iteration = 0
            while True:
                # Receive data
                reported_ee, joint_angles = self.receive_data()
                if reported_ee is None or joint_angles is None:
                    print("No data received, retrying...")
                    continue

                iteration += 1

                if len(joint_angles) != 6:
                    print(f"Expected 6 joint angles, got {len(joint_angles)}")
                    continue

                # Calculate forward kinematics
                calculated_ee = self.calculate_forward_kinematics(joint_angles)

                # Auto mode: try all transformations
                if self.coordinate_mode == 'auto':
                    print(f"\nIteration {iteration}:")
                    print(
                        f"Joint Angles (rad): {[f'{j:.4f}' for j in joint_angles]}")
                    print(
                        f"Unity reported EE:  [{reported_ee[0]:.6f}, {reported_ee[1]:.6f}, {reported_ee[2]:.6f}]")
                    print(
                        f"RTB calculated EE:  [{calculated_ee[0]:.6f}, {calculated_ee[1]:.6f}, {calculated_ee[2]:.6f}]")
                    print()

                    # Try different transformations
                    transforms = {
                        'none (direct)': reported_ee,
                        'unity_to_ros [Z,-X,Y]': self.transform_coordinates(reported_ee, 'unity_to_ros'),
                        '[X,Z,Y]': np.array([reported_ee[0], reported_ee[2], reported_ee[1]]),
                        '[Y,X,Z]': np.array([reported_ee[1], reported_ee[0], reported_ee[2]]),
                        '[Z,X,Y]': np.array([reported_ee[2], reported_ee[0], reported_ee[1]]),
                        '[-X,-Y,Z]': np.array([-reported_ee[0], -reported_ee[1], reported_ee[2]]),
                    }

                    best_error = float('inf')
                    best_mode = None

                    for name, transformed in transforms.items():
                        is_valid, error_mag, error_vec = self.validate(
                            transformed, calculated_ee)
                        status = '✓' if is_valid else '✗'
                        print(
                            f"  {status} {name:20s}: error = {error_mag:.6f}m  [{transformed[0]:.4f}, {transformed[1]:.4f}, {transformed[2]:.4f}]")

                        if error_mag < best_error:
                            best_error = error_mag
                            best_mode = name

                    print(
                        f"\n  Best transformation: {best_mode} (error: {best_error:.6f}m)")
                    print("-" * 70)
                else:
                    # Single mode validation
                    transformed_ee = self.transform_coordinates(
                        reported_ee, self.coordinate_mode)
                    is_valid, error_mag, error_vector = self.validate(
                        transformed_ee, calculated_ee)

                    # Print results
                    print(f"\nIteration {iteration}:")
                    print(
                        f"Joint Angles (rad): {[f'{j:.4f}' for j in joint_angles]}")
                    print(
                        f"Unity EE:       [{reported_ee[0]:.6f}, {reported_ee[1]:.6f}, {reported_ee[2]:.6f}]")
                    print(
                        f"Transformed EE: [{transformed_ee[0]:.6f}, {transformed_ee[1]:.6f}, {transformed_ee[2]:.6f}]")
                    print(
                        f"Calculated EE:  [{calculated_ee[0]:.6f}, {calculated_ee[1]:.6f}, {calculated_ee[2]:.6f}]")
                    print(
                        f"Error vector:   [{error_vector[0]:.6f}, {error_vector[1]:.6f}, {error_vector[2]:.6f}]")
                    print(f"Error magnitude: {error_mag:.6f}m")
                    print(
                        f"Status: {'✓ VALID' if is_valid else '✗ INVALID (exceeds tolerance)'}")
                    print("-" * 70)

        except KeyboardInterrupt:
            print("\n\nValidation stopped by user")
        finally:
            self.disconnect()

    def run_single_validation(self, end_effector, joint_angles):
        """
        Run a single validation without TCP connection (for testing).

        Args:
            end_effector (list or np.ndarray): End effector position [x, y, z]
            joint_angles (list or np.ndarray): 6 joint angles in radians
        """
        end_effector = np.asarray(end_effector)
        joint_angles = np.asarray(joint_angles)

        calculated_ee = self.calculate_forward_kinematics(joint_angles)
        is_valid, error_mag, error_vector = self.validate(
            end_effector, calculated_ee)

        print(f"Reported EE:    {end_effector}")
        print(f"Calculated EE:  {calculated_ee}")
        print(f"Error vector:   {error_vector}")
        print(f"Error magnitude: {error_mag:.6f}m")
        print(f"Status: {'✓ VALID' if is_valid else '✗ INVALID'}")

        return is_valid

    def run_ik_tests(self, num_tests=10):
        """
        Run comprehensive IK tests with random configurations.

        Args:
            num_tests (int): Number of random configurations to test
        """
        print("\n" + "=" * 70)
        print("INVERSE KINEMATICS VALIDATION TESTS")
        print("=" * 70)

        # Test 1: Zero configuration
        print("\n[Test 1] Zero Configuration Round-Trip")
        print("-" * 70)
        q_zero = np.zeros(6)
        result = self.test_ik_round_trip(q_zero)
        self._print_ik_result(result)

        # Test 2: Known good configuration
        print("\n[Test 2] Home Configuration Round-Trip")
        print("-" * 70)
        q_home = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
        result = self.test_ik_round_trip(q_home)
        self._print_ik_result(result)

        # Test 3: Random configurations
        print(f"\n[Test 3] Random Configurations ({num_tests} tests)")
        print("-" * 70)
        successes = 0
        failures = 0
        total_pos_error = 0
        max_pos_error = 0

        for i in range(num_tests):
            # Generate random joint angles within typical limits
            q_random = np.random.uniform(-np.pi, np.pi, 6)
            result = self.test_ik_round_trip(q_random)

            if result['success']:
                successes += 1
                total_pos_error += result['position_error']
                max_pos_error = max(max_pos_error, result['position_error'])
            else:
                failures += 1
                if failures <= 3:  # Only print first 3 failures
                    print(f"  Test {i+1}: FAILED - {result.get('error', 'Unknown error')}")

        print(f"\nResults: {successes}/{num_tests} successful ({successes/num_tests*100:.1f}%)")
        if successes > 0:
            print(f"Average position error: {total_pos_error/successes*1000:.3f} mm")
            print(f"Maximum position error: {max_pos_error*1000:.3f} mm")
        if failures > 0:
            print(f"Failures: {failures}")

        # Test 4: Workspace boundary tests
        print("\n[Test 4] Workspace Boundary Tests")
        print("-" * 70)
        test_positions = [
            ("Near center", np.array([0.3, 0.0, 0.3])),
            ("Extended reach", np.array([0.6, 0.2, 0.2])),
            ("High position", np.array([0.2, 0.0, 0.6])),
            ("Low position", np.array([0.3, 0.0, 0.1])),
        ]

        for name, pos in test_positions:
            success, solution, iterations = self.calculate_inverse_kinematics(pos)
            if success:
                # Verify FK
                T_verify = self.ur5.fkine(solution)
                error = np.linalg.norm(T_verify.t - pos)
                status = '✓' if error < 0.001 else '✗'
                print(f"  {status} {name:20s}: error = {error*1000:.3f} mm, iterations = {iterations}")
            else:
                print(f"  ✗ {name:20s}: IK failed (unreachable)")

        print("\n" + "=" * 70)

    def _print_ik_result(self, result):
        """Helper to print IK test results."""
        if result['success']:
            print(f"✓ SUCCESS")
            print(f"  Position error: {result['position_error']*1000:.3f} mm")
            print(f"  Joint angle error: {np.rad2deg(result['angle_error']):.3f}°")
            print(f"  Iterations: {result['iterations']}")
        else:
            print(f"✗ FAILED: {result.get('error', 'Unknown error')}")

    def test_ik_server(self, server_host='127.0.0.1', server_port=5010, num_tests=5):
        """
        Test the IK server by sending requests and validating responses.

        Args:
            server_host (str): IK server host
            server_port (int): IK server port
            num_tests (int): Number of test requests to send
        """
        print("\n" + "=" * 70)
        print(f"IK SERVER TEST - {server_host}:{server_port}")
        print("=" * 70)

        try:
            # Connect to IK server
            ik_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ik_socket.settimeout(5.0)
            ik_socket.connect((server_host, server_port))
            print(f"✓ Connected to IK server")

            successes = 0
            total_time = 0

            for i in range(num_tests):
                # Generate random target
                target_pos = np.random.uniform([0.2, -0.3, 0.1], [0.6, 0.3, 0.6])
                target_rot = np.array([0, 0, 0, 1])  # Identity quaternion
                current_angles = np.random.uniform(-np.pi, np.pi, 6)

                print(f"\n[Test {i+1}] Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")

                # Send IK request (Command 1: SolveIK)
                start_time = time.time()

                # Send command type
                ik_socket.sendall(struct.pack('B', 1))

                # Send target position (3 doubles)
                ik_socket.sendall(struct.pack('<3d', *target_pos))

                # Send target rotation (4 doubles)
                ik_socket.sendall(struct.pack('<4d', *target_rot))

                # Send current angles (6 doubles)
                ik_socket.sendall(struct.pack('<6d', *current_angles))

                # Read response
                success_byte = ik_socket.recv(1)
                if len(success_byte) == 0:
                    print("  ✗ Server disconnected")
                    break

                success_flag = struct.unpack('B', success_byte)[0]

                if success_flag == 1:
                    # Read solution (6 doubles = 48 bytes)
                    solution_data = b''
                    while len(solution_data) < 48:
                        chunk = ik_socket.recv(48 - len(solution_data))
                        if not chunk:
                            break
                        solution_data += chunk

                    solution = np.array(struct.unpack('<6d', solution_data))
                    elapsed = time.time() - start_time
                    total_time += elapsed

                    # Validate solution with FK
                    T_result = self.ur5.fkine(solution)
                    pos_error = np.linalg.norm(T_result.t - target_pos)

                    status = '✓' if pos_error < 0.01 else '✗'
                    print(f"  {status} Solution received in {elapsed*1000:.1f} ms")
                    print(f"     Position error: {pos_error*1000:.3f} mm")
                    print(f"     Joint solution: {np.rad2deg(solution).round(1)}")

                    if pos_error < 0.01:
                        successes += 1
                else:
                    print("  ✗ Server returned no solution")

            print(f"\n{'-' * 70}")
            print(f"Results: {successes}/{num_tests} successful ({successes/num_tests*100:.1f}%)")
            if successes > 0:
                print(f"Average solve time: {total_time/successes*1000:.1f} ms")

            ik_socket.close()
            print("\n" + "=" * 70)

        except socket.timeout:
            print("✗ Connection timeout - is the IK server running?")
        except ConnectionRefusedError:
            print(f"✗ Connection refused - IK server not running on {server_host}:{server_port}")
        except Exception as e:
            print(f"✗ Error: {e}")
            import traceback
            traceback.print_exc()


def main():
    """Main entry point"""
    # Parse command line arguments
    import argparse

    parser = argparse.ArgumentParser(
        description='UR5 Forward and Inverse Kinematics Validator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test FK continuously from Unity
  python ur5_fk_validator.py --host localhost --port 5005

  # Test IK locally with 20 random configurations
  python ur5_fk_validator.py --test-ik --num-tests 20

  # Test IK server
  python ur5_fk_validator.py --test-ik-server --ik-port 5010

  # Run all tests
  python ur5_fk_validator.py --test-ik --test-ik-server
        """)

    # FK options
    parser.add_argument('--host', type=str, default='localhost',
                        help='Unity FK server host (default: localhost)')
    parser.add_argument('--port', type=int, default=5005,
                        help='Unity FK server port (default: 5005)')
    parser.add_argument('--tolerance', type=float, default=0.01,
                        help='Position error tolerance in meters (default: 0.01)')

    # Test modes
    parser.add_argument('--test-fk', action='store_true',
                        help='Run single FK test without TCP connection')
    parser.add_argument('--test-ik', action='store_true',
                        help='Run IK validation tests (local, no server needed)')
    parser.add_argument('--test-ik-server', action='store_true',
                        help='Test the Python IK server')

    # IK test options
    parser.add_argument('--num-tests', type=int, default=10,
                        help='Number of random IK tests to run (default: 10)')
    parser.add_argument('--ik-host', type=str, default='127.0.0.1',
                        help='IK server host (default: 127.0.0.1)')
    parser.add_argument('--ik-port', type=int, default=5010,
                        help='IK server port (default: 5010)')

    args = parser.parse_args()

    # Create validator
    validator = UR5FKValidator(
        host=args.host,
        port=args.port,
        tolerance=args.tolerance
    )

    # Determine what tests to run
    run_any_test = args.test_fk or args.test_ik or args.test_ik_server

    if args.test_fk:
        # FK test mode: validate a known configuration
        print("\n" + "=" * 70)
        print("FORWARD KINEMATICS TEST")
        print("=" * 70)
        # Zero configuration (all joints at 0)
        test_ee = np.array([0.0, -0.3915, 0.8116])
        test_joints = np.array([0, 0, 0, 0, 0, 0])
        validator.run_single_validation(test_ee, test_joints)

    if args.test_ik:
        # IK test mode: run comprehensive IK tests
        validator.run_ik_tests(num_tests=args.num_tests)

    if args.test_ik_server:
        # IK server test mode: test the Python IK server
        validator.test_ik_server(
            server_host=args.ik_host,
            server_port=args.ik_port,
            num_tests=args.num_tests
        )

    if not run_any_test:
        # Default: continuous FK validation from Unity
        print("\nStarting continuous FK validation from Unity...")
        print("Press Ctrl+C to stop\n")
        validator.run_continuous_validation()


if __name__ == '__main__':
    main()
