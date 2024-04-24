import numpy as np

from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster


class SimulatedRobotBase:
    WATCHDOG_FREQUENCY_HZ = 5

    def __init__(
        self, uuid, rigid_body_label, node, initial_position, initial_orientation
    ):
        super().__init__()
        assert len(initial_orientation) == 3
        assert len(initial_position) == 3

        self.orientation = R.from_euler("xyz", initial_orientation)
        self.position = np.array(initial_position, dtype=float)

        self.node = node
        self.uuid = uuid
        self.rigid_body_label = rigid_body_label

        self.emergency_stop_srv = self.node.create_service(
            SetBool,
            f"/{self.rigid_body_label}/emergency_stop",
            self.emergency_stop,
        )

        self.tf_publisher = TransformBroadcaster(node=self.node)

        self.watchdog = None
        self.stopped = False

    def emergency_stop(self, req, resp):
        self.node.get_logger().debug(f"Received emergency stop request {req}")
        self.stopped = req.data
        resp.success = True
        return resp

    def reset_watchdog(self):
        # Reset the watchdog. The watchdog only starts running after it was reset once.
        if self.watchdog is None:
            self.watchdog = self.node.create_timer(
                1 / self.WATCHDOG_FREQUENCY_HZ, self.stop
            )
        self.watchdog.reset()

    def step(self, dt):
        # dt: time step
        raise NotImplementedError()

    def stop(self):
        # Called by watchdog if wasn't reset in time
        raise NotImplementedError()

    def publish_tf(self):
        tf = TransformStamped()
        tf.header.frame_id = "map_ned"
        tf.header.stamp = self.node.get_clock().now().to_msg()
        tf.child_frame_id = self.rigid_body_label

        tf.transform.translation.x = self.position[0]
        tf.transform.translation.y = self.position[1]
        tf.transform.translation.z = self.position[2]

        orientation = self.orientation.as_quat()
        tf.transform.rotation.x = orientation[0]
        tf.transform.rotation.y = orientation[1]
        tf.transform.rotation.z = orientation[2]
        tf.transform.rotation.w = orientation[3]

        self.tf_publisher.sendTransform(tf)
