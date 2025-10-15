import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CheckGripperReverse(Node):
    def __init__(self):
        super().__init__('gripper_commander_reverse_node')

        self.pal_gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_left_controller/joint_trajectory',
            10)

        self.current_position = 0.95
        self.step = -0.05
        self.min_position = 0.0

        timer_period = 5.0  # secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Nodo avviato. Eseguo chiusura gripper da 0.95 a 0.0.')

    def timer_callback(self):
        # Usiamo una piccola tolleranza (1e-6) per confronti sicuri con i float
        if self.current_position < self.min_position - 1e-6:
            self.get_logger().info('Posizione minima raggiunta. Interruzione della pubblicazione.')
            self.timer.cancel()  # Ferma il timer
            return

        traj_msg = JointTrajectory()

        traj_msg.joint_names = ['gripper_left_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.current_position]
        point.time_from_start = Duration(sec=1, nanosec=0)

        traj_msg.points.append(point)

        self.pal_gripper_pub.publish(traj_msg)
        self.get_logger().info(f'Pubblicata posizione gripper: {self.current_position:.2f}')

        self.current_position += self.step

def main(args=None):
    rclpy.init(args=args)

    gripper_commander_node = CheckGripperReverse()

    try:
        rclpy.spin(gripper_commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_commander_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()