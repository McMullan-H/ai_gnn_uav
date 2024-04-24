import rclpy

import numpy as np
from scipy.spatial.transform import Rotation as R

from ctrl_msgs.msg import MinicarControl
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class Minicar(SimulatedRobotBase):
    CAR_LENGTH = 0.15  # m
    MAX_V = 1.0  # m/s
    MAX_STEER = np.pi / 10  # rad

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.control = MinicarControl()
        self.control_subscription = self.node.create_subscription(
            MinicarControl, f"/{self.uuid}/cmd_vel", self.control_callback, 1
        )
        self.orientation_offset = R.from_euler("z", np.pi / 2)

    def control_callback(self, ctrl: MinicarControl):
        self.reset_watchdog()
        self.control = ctrl

    def stop(self):
        self.control = MinicarControl()

    def step(self, dt):
        v_clip = np.clip(self.control.velocity, 0.0, self.MAX_V)
        steer_clip = np.clip(self.control.steering, -self.MAX_STEER, self.MAX_STEER)
        self.position += self.orientation.apply(np.array([v_clip, 0, 0]) * dt)
        self.orientation *= R.from_euler(
            "z", np.tan(steer_clip) / self.CAR_LENGTH * v_clip * dt
        )


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_minicar", r"/minicar_\d+/", Minicar)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
