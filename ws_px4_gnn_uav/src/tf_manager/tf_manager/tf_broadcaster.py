import math
import sys

from geometry_msgs.msg import TransformStamped
from gnn_uav_msgs.msg import RelativePosition

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
    
class TFBroadcaster(Node):

	def __init__(self):
		
		# Declare frame message
		self.t = TransformStamped()
		
		super().__init__("tf_broadcaster")

		rclpy.logging.get_logger("rclpy").info(f"Node: tf_broadcaster started")
		
		'''
		# PX4 compatible QoS profile for subscribers
		qos_profile = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			durability=DurabilityPolicy.TRANSIENT_LOCAL,
			history=HistoryPolicy.KEEP_LAST,
			depth=1
		)
		'''
		
		self.declare_parameter("frame_id", "invalid_frame")
		self.declare_parameter("uuid", "invalid_uuid")
		self.declare_parameter("agent", 0)
		self.declare_parameter("sim_en", 0)

		self.frame_id = self.get_parameter("frame_id").value
		self.uuid = self.get_parameter("uuid").value
		self.agent = self.get_parameter("agent").value
		self.sim_en = self.get_parameter("sim_en").value
		
		self.tf_broadcaster = TransformBroadcaster(self)

		self.subscription = self.create_subscription(
			RelativePosition,
			f"/uav_{self.agent}/relative_position",
			self.tf_callback,
			10)
		self.subscription
		
		self.timer = self.create_timer(0.1, self.timer_callback)
	
	def tf_callback(self, msg):
		self.t.transform.translation.x = float(msg.xn)
		self.t.transform.translation.y = float(msg.ye)
		self.t.transform.translation.z = 0.0
		
		self.t.transform.rotation.x = 0.0
		self.t.transform.rotation.y = 0.0
		self.t.transform.rotation.z = 0.0
		self.t.transform.rotation.w = 1.0

		'''
		self.t.transform.rotation.x = float(msg.q[0])
		self.t.transform.rotation.y = float(msg.q[1])
		self.t.transform.rotation.z = float(msg.q[2])
		self.t.transform.rotation.w = float(msg.q[3])
		'''
		
	def timer_callback(self):
		self.t.header.stamp = self.get_clock().now().to_msg()
		self.t.header.frame_id = self.frame_id
		self.t.child_frame_id = self.uuid
	
		self.tf_broadcaster.sendTransform(self.t)
		
def main():
	
	rclpy.init()
	
	try:
		rclpy.spin(TFBroadcaster())
	except KeyboardInterrupt:
		pass
	
	rclpy.shutdown()
