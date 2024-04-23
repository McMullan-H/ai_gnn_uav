# ai_gnn_uav

## Instructions

Only relevant folders are included in this repository due to PX4 firmware being a submodule, and building requiring that the firmware is cloned from the main repositry for proper versioning. To avoid making a new branch, which we do not have permission for anyways, please do the following;

1. Clone PX4 V1.14 from the main repository.
'git clone --recursive git@github.com:PX4/PX4-Autopilot.git -b release/1.14'

2. Replace the "mavlink" and "uxrce-dds-client" modules with the corrisponding modules from this repository.
'PX4-Autopilot/src/modules/'

3. Replace the uORB message definitions "msg".
'PX4-Autopilot/msg/'

4. Increase the uORB message enumeration type width from 8-bit to 16-bit within the generation template, otherwise it will not allow compilation of more than 255 uORB topics.
'PX4-Autopilot/Tools/msg/templates/uorb/uORBTopics.hpp.em'
'Line 65: enum class ORB_ID : uint8_t { -> enum class ORB_ID : uint16_t {'

5. Install the build requirements which will include the Gazebo Classic simulator.
'./Tools/setup/ubuntu.sh'

6. You should be good to go. Make the software-in-the-loop simulation from the top-level directory. **Note: DON'T FOLLOW THE INSTRUCTIONS FROM THE PX4 ROS2 GUIDE**. For multi-agent simulation a different build script is required.
'DONT_RUN=1 make px4_sitl gazebo-classic'

8. Build both the ROS2 pub/sub nodes and the GNN multi agent passage framework using 'colcon build'. This is assuming you already have a ROS distribution installed. If not, this was tested using Foxy. Other versions are not guarenteed to work! The GNN code will fail on the first build. This is a known issue with the RVO package. However it is not problematic.

9. From the top-level directory of the PX4 firmware, run the Gazebo simulation using the launch script, entering the number of agents as an argument.
'Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n <number_of_agents>'

10. In another terminal window, from the ROS2 pub/sub workspace, source the overlay and run the launch file. This launch file will contain a number of parameters that you have available to tweak.
'source install/local_setup.bash'
'ros2 launch launch/gnn_uav.launch.py'

11. In another terminal window, from the GNN multi agent passage workspace, source the overlay and launch the decentralized GNN policy.
'source install/local_setup.bash'
'ros2 launch src/passage_gnn_simple/launch/decentralized_passage_robomaster.launch.py'

**Note: Building is assuming that you are running this on Ubuntu 20.04 Focal, as this is the final version that supports Gazebo Classic**
