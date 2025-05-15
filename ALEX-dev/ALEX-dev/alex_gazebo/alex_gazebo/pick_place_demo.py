"""
 _   _                      _ _____                     _            
| \ | | ___ _   _ _ __ __ _| |  ___|__  _   _ _ __   __| |_ __ _   _ 
|  \| |/ _ \ | | | '__/ _` | | |_ / _ \| | | | '_ \ / _` | '__| | | |
| |\  |  __/ |_| | | | (_| | |  _| (_) | |_| | | | | (_| | |  | |_| |
|_| \_|\___|\__,_|_|  \__,_|_|_|  \___/ \__,_|_| |_|\__,_|_|   \__, |
                                                               |___/ 
"""

# ROS Imports
import rclpy
from rclpy.node import Node
# ROS Interface Imports
from std_msgs.msg import Float64MultiArray
from dsr_msgs2.srv import MoveJoint
# Python Imports
from ament_index_python.packages import get_package_share_directory
import subprocess
import time
from pathlib import Path

class PickPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_place_demo')
        
        # Path to the Gazebo model
        gazebo_path = Path(get_package_share_directory('alex_gazebo'))
        self.pick_object = gazebo_path / "models" / "simple_box"

        # Predefined joint positions for various steps
        self.joint_position = {
            "initial pose": [90.0, 90.0, 0.0, 0.0, 0.0, 0.0],
            "lift item": [90.0, 85.0, 0.0, 0.0, 0.0, 0.0],
            "move item": [0.0, 85.0, 0.0, 0.0, 0.0, 0.0],
            "keep item": [0.0, 90.0, 0.0, 0.0, 0.0, 0.0],
            "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

        # Predefined gripper positions
        self.gripper_position = {
            "fully open": 0.01,
            "fully closed": 0.037
        }

        # Create a publisher to control the gripper
        self.publisher = self.create_publisher(Float64MultiArray, '/dsr01/gz/gripper_controller/commands', 10)

        # Start the demo on initialization
        self.demo_start()

    def demo_start(self):
        """Starts the demo sequence by moving the robot, handling the box, and controlling the gripper."""
        try:
            # Open the gripper at the beginning
            self.publish_gripper(self.gripper_position["fully open"])

            # Step 1: Move to the initial pose
            if not self.call_move_joint_service(self.joint_position["initial pose"]):
                return

            # Step 2: Spawn the box in Gazebo
            if not self.run_ignition_service():
                return

            # Step 3: Close the gripper to pick the object
            self.publish_gripper(self.gripper_position["fully closed"])

            # Step 4: Move through a sequence of positions
            if not self.call_move_joint_service(self.joint_position["lift item"]):
                return
            if not self.call_move_joint_service(self.joint_position["move item"]):
                return
            if not self.call_move_joint_service(self.joint_position["keep item"]):
                return

            # Step 5: Open the gripper to release the object
            self.publish_gripper(self.gripper_position["fully open"])

            # Step 6: Move to the home position
            if not self.call_move_joint_service(self.joint_position["home"]):
                return

            # Step 7: Remove the box from Gazebo
            if not self.remove_ignition_service():
                return

            self.get_logger().info("Demo Over: All steps completed successfully.")

        except Exception as e:
            self.get_logger().error(f"Demo failed due to error: {e}")
        finally:
            self.shutdown_demo()

    def call_move_joint_service(self, position, velocity=30.0, acceleration=30.0):
        """Calls the move_joint service to move the robot to a specified position."""
        client = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the move_joint service...')

        # Prepare and send request
        request = MoveJoint.Request()
        request.pos = position
        request.vel = velocity
        request.acc = acceleration

        # Wait for the result
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Successfully moved to the requested position.")
                time.sleep(5)
                return True
            else:
                self.get_logger().error("Failed to move the robot.")
                return False
        else:
            self.get_logger().error("No result received from move_joint service.")
            return False

    def run_ignition_service(self):
        """Spawns a box in the Ignition Gazebo world."""
        command = (
            'ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory '
            '--reptype ignition.msgs.Boolean --timeout 300 --req \'sdf_filename: '
            f'"{str(self.pick_object)}", '
            'pose: {position: {x:0.0, y:1.06, z:10}}, '
            f'name: "{self.pick_object.name}"\''
        )

        self.get_logger().info(f"Running command: {command}")
        process = subprocess.run(command, shell=True, capture_output=True, text=True)

        if "data: true" in process.stdout:
            self.get_logger().info("Box spawned successfully in Gazebo.")
            time.sleep(2)
            return True
        else:
            self.get_logger().error(f"Failed to spawn box: {process.stderr}")
            return False

    def remove_ignition_service(self):
        """Removes the box from Ignition Gazebo."""
        command = (
            'ign service -s /world/empty/remove '
            '--reqtype ignition.msgs.Entity '
            '--reptype ignition.msgs.Boolean '
            '--timeout 300 '
            f'--req \'name: "{self.pick_object.name}" type: MODEL\''
        )

        self.get_logger().info(f"Running command: {command}")
        process = subprocess.run(command, shell=True, capture_output=True, text=True)

        if "data: true" in process.stdout:
            self.get_logger().info("Box removed successfully from Gazebo.")
            return True
        else:
            self.get_logger().error(f"Failed to remove box: {process.stderr}")
            return False

    def publish_gripper(self, gripper_data):
        """Publishes the gripper command to open/close the gripper."""
        msg = Float64MultiArray()
        msg.data = [gripper_data]

        self.publisher.publish(msg)
        self.get_logger().info(f"Gripper command published: {gripper_data}")

        # Sleep to ensure gripper operation is completed
        time.sleep(3)

    def shutdown_demo(self):
        """Shuts down the demo gracefully by destroying the node."""
        self.get_logger().info("Shutting down demo gracefully.")
        self.destroy_node()
        exit(0)


def main(args=None):
    """Main entry point for the PickPlaceDemo."""
    rclpy.init(args=args)
    node = PickPlaceDemo()

    # Spin the node to keep it alive
    rclpy.spin(node)


if __name__ == '__main__':
    main()
