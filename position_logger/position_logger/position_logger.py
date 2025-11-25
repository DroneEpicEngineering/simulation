#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint
import csv
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PositionLogger(Node):
    def __init__(self):
        super().__init__('position_logger')

        self.csv_file = os.path.join(os.getcwd(), 'positions_log.csv')
        with open(self.csv_file, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow([
                'sim_time', 
                'drone_x', 'drone_y', 'drone_z',
                'target_x', 'target_y', 'target_z'
            ])

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

#TODO: SPRAWDZIĆ CZY OK
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.drone_callback,
            qos
        )

        self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.target_callback,
            qos
        )

        # TODO: timer
        self.create_timer(0.1, self.log_positions)

    def drone_callback(self, msg: VehicleLocalPosition):
        self.drone_pose = msg

    def target_callback(self, msg: TrajectorySetpoint):
        self.target_pose = msg

    def log_positions(self):
        if self.drone_pose is None or self.target_pose is None:
            return  

        # TODO: Sprawdizć jak to tak naprawdę liczy czas
        sim_time = self.get_clock().now().nanoseconds * 1e-9

        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f, delimiter=';')
            writer.writerow([
                sim_time,
                self.drone_pose.x,
                self.drone_pose.y,
                self.drone_pose.z,
                self.target_pose.x,
                self.target_pose.y,
                self.target_pose.z
            ])

#info do terminala
        self.get_logger().info(
            f"Logged at {sim_time:.2f}s: "
            f"Drone ({self.drone_pose.x:.2f},{self.drone_pose.y:.2f},{self.drone_pose.z:.2f}), "
            f"Target ({self.target_pose.x:.2f},{self.target_pose.y:.2f},{self.target_pose.z:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PositionLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
