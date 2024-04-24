import rclpy
from geometry_msgs.msg import Twist
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node

from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class CrazyFlie(SimulatedRobotBase):
    MAX_V_LINEAR_X_M_S = 1.5
    MAX_V_LINEAR_Y_M_S = 1.5
    MAX_V_LINEAR_Z_M_S = 1.0

    MAX_V_ROT_Z_RAD_S = 20.0

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

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

        self.position += (
            np.array(
                [
                    np.clip(
                        self.velocity.vx,
                        -self.MAX_V_LINEAR_X_M_S,
                        self.MAX_V_LINEAR_X_M_S,
                    ),
                    np.clip(
                        self.velocity.vy,
                        -self.MAX_V_LINEAR_Y_M_S,
                        self.MAX_V_LINEAR_Y_M_S,
                    ),
                    np.clip(
                        self.velocity.vz,
                        -self.MAX_V_LINEAR_Z_M_S,
                        self.MAX_V_LINEAR_Z_M_S,
                    ),
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
                        -self.velocity.omega,
                        -self.MAX_V_ROT_Z_RAD_S,
                        self.MAX_V_ROT_Z_RAD_S,
                    ),
                ]
            )
            * dt,
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_crazyflie", r"/crazyflie_\d+/", CrazyFlie)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
