from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit

def generate_launch_description():
	
	sim_params = {
		"uuids": ["uav_0", "uav_1", "uav_2", "uav_3", "uav_4"],
		"uav_0_initial_position": [-2.0, 2.0, 0.0],
		"uav_1_initial_position": [-2.0, -2.0, 0.0],
		"uav_2_initial_position": [-3.0, -1.0, 0.0],
		"uav_3_initial_position": [-1.0, 0.0, 0.0],
		"uav_4_initial_position": [-3.0, 1.0, 0.0],
	}
    
	ld = LaunchDescription(
		[
			Node(
				package="simple_simulator",
				namespace="/sim",
				executable="robomaster",
				name="simulation",
				parameters=[sim_params],
			),
			Node(
    			package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "1.57", "0.0", "3.14", "map", "map_ned"],
    		)
    	]
    )
    
	return ld
