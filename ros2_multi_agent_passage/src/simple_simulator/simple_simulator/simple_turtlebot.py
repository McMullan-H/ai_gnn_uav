import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class Turtlebot(SimulatedRobotBase):
    MAX_LATERAL_V_M_S = 0.2
    MAX_ANGULAR_V_RAD_S = np.pi * 0.75

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )
        self.orientation_offset = R.from_euler("z", -3 * np.pi / 2)

        self.velocity = Twist()
        self.velocity_subscription = self.node.create_subscription(
            Twist, f"/{self.uuid}/cmd_vel", self.velocity_callback, 1
        )

    def velocity_callback(self, vel):
        self.reset_watchdog()
        self.velocity = vel

    def stop(self):
        self.velocity = Twist()

    def step(self, dt):
        self.position += self.orientation.apply(
            np.array(
                [
                    np.clip(
                        self.velocity.linear.x,
                        -self.MAX_LATERAL_V_M_S,
                        self.MAX_LATERAL_V_M_S,
                    ),
                    0,
                    0,
                ]
            )
            * dt
        )
        self.orientation *= R.from_euler(
            "xyz",
            np.array(
                [
                    0,
                    0,
                    np.clip(
                        self.velocity.angular.z,
                        -self.MAX_ANGULAR_V_RAD_S,
                        self.MAX_ANGULAR_V_RAD_S,
                    ),
                ]
            )
            * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_turtlebot", r"/turtlebot_\d+/", Turtlebot)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
