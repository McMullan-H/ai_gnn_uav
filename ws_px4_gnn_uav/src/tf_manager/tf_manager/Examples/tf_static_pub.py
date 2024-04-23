import math
import sys

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

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

class TFStaticPublisher(Node):
	'''
	Broadcast a transform that never changes.
	
	This is used as a base frame in the implementation.
	'''
	
	def __init__(self):
		super().__init__("tf_static_pub")
		
		self.tf_static_broadcaster = StaticTransformBroadcaster(self)
		
		# Publish the static transform once at startup
		t = TransformStamped()
		
		t.header.stamp = self.get_clock().now().to_msg()
		t.header.frame_id = "map"
		t.child_frame_id = "map_ned"
		
		t.transform.translation.x = float(0)
		t.transform.translation.y = float(0)
		t.transform.translation.z = float(0)
		
		quat = quaternion_from_euler(float(0), float(0), float(0))
		
		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]
		
		self.tf_static_broadcaster.sendTransform(t)
		
def main():
	logger = rclpy.logging.get_logger("logger")
	print("[Node] tf_static_pub: Starting...")
	
	rclpy.init()
	
	try:
		rclpy.spin(TFStaticPublisher())
	except KeyboardInterrupt:
		pass
		
	rclpy.shutdown()
