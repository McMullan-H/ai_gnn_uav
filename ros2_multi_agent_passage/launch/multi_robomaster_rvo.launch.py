from launch import LaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription


def generate_launch_description():
    uuids = [
        "uav_0",
        "uav_1",
        "uav_2",
        "uav_3",
        "uav_4",
    ]
    ld = []
    for uuid in uuids:
        ld.append(
            GroupAction(
                [
                    PushRosNamespace(uuid),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            "launch/freyja_robomaster.launch.yaml"
                        ),
                        launch_arguments=[("uuid", uuid)],
                    ),
                ]
            )
        )
    return LaunchDescription(ld)
