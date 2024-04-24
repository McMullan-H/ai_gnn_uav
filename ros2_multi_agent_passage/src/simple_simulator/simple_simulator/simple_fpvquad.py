import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.qos import qos_profile_sensor_data
from freyja_msgs.msg import CtrlCommand
from .simulated_robot import SimulatedRobotBase
from .simple_simulator import SimpleSimulator


class FpvQuad(SimulatedRobotBase):
    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__(
            uuid, rigid_body_label, node, initial_position, initial_orientation
        )

        self.control = CtrlCommand()
        self.v_ned = np.zeros(3)
        self.a_ned = np.zeros(3)
        self.control_subscription = self.node.create_subscription(
            CtrlCommand,
            f"/{self.uuid}/rpyt_command",
            self.control_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def control_callback(self, control: CtrlCommand):
        self.reset_watchdog()
        self.control = control

    def stop(self):
        self.control = CtrlCommand()

    def step(self, dt):
        self.orientation = R.from_euler(
            "xyz",
            np.array(
                [
                    self.control.roll,
                    self.control.pitch,
                    self.orientation.as_euler("xyz")[2] + self.control.yaw * dt,
                ]
            ),
        )
        roll = self.control.roll
        pitch = self.control.pitch
        _, _, yaw = self.orientation.as_euler("xyz")

        rot_force = np.array(
            [
                np.cos(roll) * np.sin(pitch) * np.cos(yaw) + np.sin(roll) * np.sin(yaw),
                np.cos(roll) * np.sin(pitch) * np.sin(yaw) - np.sin(roll) * np.cos(yaw),
                np.cos(roll) * np.cos(pitch),
            ]
        )
        m = 0.85  # kg
        g = 9.81  # m/s^2
        f_ned = -rot_force * self.control.thrust + [0, 0, m * g]
        self.a_ned = f_ned / m
        self.v_ned += self.a_ned * dt
        self.position += self.v_ned * dt
        if self.position[2] > 0.0:
            self.v_ned = np.zeros(3)
            self.position[2] = 0.0


def main(args=None):
    rclpy.init(args=args)
    publisher = SimpleSimulator("simple_fpvquad", r"/fpvquad_\d+/", FpvQuad)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
