from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, Shutdown, LogInfo
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Crash on node failure
    # sim_params = {"uuids": []}
    # for i, px in enumerate([-1.5]): #, 0.05, 1.02]):
    #    sim_params["uuids"].append(f"robomaster_{i}")
    #    sim_params[f"robomaster_{i}_initial_position"] = [-2.0, px, 0.0]

    sim_params = {
        "uuids": ["uav_0", "uav_1", "uav_2", "uav_3", "uav_4"],
        "uav_0_initial_position": [-2.0, -2.0, -0.0],
        "uav_1_initial_position": [-3.5, -1.0, -0.0],
        "uav_2_initial_position": [-4.0, 0.0, -0.0],
        "uav_3_initial_position": [-3.5, 1.0, -0.0],
        "uav_4_initial_position": [-2.0, 2.0, -0.0],
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
                package="rviz2",
                namespace="/rviz",
                executable="rviz2",
                name="rviz",
                arguments=["-d", "src/simple_simulator/rviz/sim.rviz"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "1.57", "0.0", "3.14", "map", "map_ned"],
            ),
        ]
    )
    shutdown = Shutdown(reason="Node failure, stopping launch...")

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                on_exit=[LogInfo(msg=["A Node crashed, shutting down..."]), shutdown]
            )
        )
    )
    return ld
