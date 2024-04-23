#include "trajectory_setpoint_pub.hpp"
#include "offboard_control_mode_pub.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <iostream>

class UAVControl : public rclcpp::Node {
public:
	
	UAVControl() : Node("uav_control"){}

};

int main(int argc, char *argv[]){
	/*	argv[1] : Number of agents in network
	*/

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[Node] uav_control: Starting...");
	
	if(argc < 2){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "[Node] uav_control: Expected at least one argument. Exiting...");
		return 1;
	}
	
	// Convert from char to int
	int agent_n;
	int sim_en;
	sscanf(argv[1], "%d", &agent_n);
	sscanf(argv[2], "%d", &sim_en);
	
	if(agent_n > 32){
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
			"[Node] uav_control: Error: A maximum of 32 agents permitted within the network at this time.");
		return 1;
	}
	
	rclcpp::init(argc, argv);
	
	// Spin multiple nodes
	rclcpp::executors::MultiThreadedExecutor executor;
	
	char node_name[32];
	
	/***********************************************************************************************/
	// Instantiating publisher/subscriber nodes
	
	// std::shared_ptr<UAVControl> node_uav_control = std::make_shared<UAVControl>();
	// executor.add_node(node_uav_control);

	std::vector<std::shared_ptr<OffboardControlModePub>> node_offboard_control_mode_pub;

	if(sim_en){

		for(int x = 0; x < agent_n; x++){
			std::sprintf(node_name, "offboard_control_mode_pub_%d", x);
			node_offboard_control_mode_pub.push_back(std::make_shared<OffboardControlModePub>(node_name, x, sim_en));
			executor.add_node(node_offboard_control_mode_pub[x]);
		}

	} else{
		node_offboard_control_mode_pub.push_back(std::make_shared<OffboardControlModePub>((char*)"offboard_control_mode_pub", 0, sim_en));
		executor.add_node(node_offboard_control_mode_pub[0]);
	}

	std::vector<std::shared_ptr<TrajectorySetpointPub>> node_trajectory_setpoint_pub;
	
	for(int x = 0; x < agent_n; x++){
		std::sprintf(node_name, "trajectory_setpoint_pub_%d", x);
		node_trajectory_setpoint_pub.push_back(std::make_shared<TrajectorySetpointPub>(node_name, x, sim_en));
		executor.add_node(node_trajectory_setpoint_pub[x]);
	}
	
	/***********************************************************************************************/
	
	executor.spin();
	
	rclcpp::shutdown();

	return 0;
}
