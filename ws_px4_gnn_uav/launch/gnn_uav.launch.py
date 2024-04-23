from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	sim_en = 1
	frame_id = "map"        # ENU reference frame
	hover_height = 3.0		# Operating height (m)
	rise_rate = 2.0			# Rising rate (m/s)
	jitter_rate = 0.1		# Jitter rate (about hover threshold) (m/s)
	hover_thresh = 0.05		# Hover setpoint boundary threshold (m)
	hover_yaw = -1.57 		# Hover yaw (rad)
	uuids = [
		"uav_0",
		"uav_1",
		"uav_2",
		"uav_3",
		"uav_4"
	]

	trajectory_setpoint_params = [{
		"sim_en" : sim_en,
		"hover_height" : hover_height,
		"hover_yaw" : hover_yaw,
		"hover_thresh" : hover_thresh,
		"rise_rate" : rise_rate,
		"jitter_rate" : jitter_rate,
        "uav_0_initial_position": [-2.0, -2.0, -0.0],
        "uav_1_initial_position": [-3.5, -1.0, -0.0],
        "uav_2_initial_position": [-4.0, 0.0, -0.0],
        "uav_3_initial_position": [-3.5, 1.0, -0.0],
        "uav_4_initial_position": [-2.0, 2.0, -0.0],
	}]

	ld = [
			Node(
				package = "tf2_ros",
				executable = "static_transform_publisher",
				arguments = ["0", "0", "0", "1.57", "0.0", "3.14", frame_id, "map_ned"]
			)
	]

	if(sim_en > 0):

		n = 0
		for uuid in uuids:
			trajectory_setpoint_params[0]["agent"] = n

			ld.append(
				Node(
					package = "gnn_uav",
					executable = "offboard_control_mode_pub",
					namespace = uuid,
					parameters = [{
						"sim_en" : sim_en,
						"agent" : n
					}]
				)
			)

			ld.append(
				Node(
					package = "gnn_uav",
					executable = "trajectory_setpoint_pub",
					namespace = uuid,
					parameters = trajectory_setpoint_params
				)
			)

			ld.append(
				Node(
					package = "tf_manager",
					executable = "tf_broadcaster",
					namespace = uuid,
					parameters = [{
						"sim_en" : sim_en,
						"frame_id" : frame_id,
						"uuid" : uuid,
						"agent" : n
					}]
				)
			)

			n += 1

	else:

		ld.append(
			Node(
				package = "gnn_uav",
				executable = "offboard_control_mode_pub",
				parameters = [{
					"sim_en" : sim_en,
					"agent" : 0
				}]
			)
		)

		n = 0
		for uuid in uuids:
			trajectory_setpoint_params[0]["agent"] = n

			ld.append(
				Node(
					package = "gnn_uav",
					executable = "trajectory_setpoint_pub",
					namespace = uuid,
					parameters = trajectory_setpoint_params
				)
			)

			ld.append(
				Node(
					package = "tf_manager",
					executable = "tf_broadcaster",
					namespace = uuid,
					parameters = [{
						"sim_en" : sim_en,
						"frame_id" : frame_id,
						"uuid" : uuid,
						"agent" : n
					}]
				)
			)

			n += 1

	return LaunchDescription(ld)
