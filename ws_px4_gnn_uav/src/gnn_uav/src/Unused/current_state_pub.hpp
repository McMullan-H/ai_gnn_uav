#include <rclcpp/rclcpp.hpp>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "freyja_msgs/msg/current_state.hpp"

#include <string>
#include <chrono>

// This just captures the vehicle_odometry message from PX4 and translates this into a current_state
// message for the Freyja state manager module

using namespace std::chrono_literals;
using std::placeholders::_1;

class CurrentStatePub : public rclcpp::Node {
public:
	CurrentStatePub(char* node_name, int agent_number) : Node(node_name){

		RCLCPP_INFO(this->get_logger(), "Node: %s started", node_name);

		this->declare_parameter("param_frame_id", "invalid_frame");
		std::string frame_id_str = this->get_parameter("param_frame_id").as_string();

		frame_id = frame_id_str.data();

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		char current_state_topic[32];
		std::sprintf(current_state_topic, "/uav_%d/current_state", agent_number);

		char vehicle_odometry_topic[32];

		if(agent_number > 0){
			std::sprintf(vehicle_odometry_topic, "/fmu/out/vehicle_odometry_%d", agent_number);
		} else{
			char str[] = "/fmu/out/vehicle_odometry";
			memcpy(vehicle_odometry_topic, str, sizeof(str));
		}
		
		vehicle_odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
			vehicle_odometry_topic, qos, std::bind(&CurrentStatePub::vehicle_odometry_callback, this, _1));
			
		current_state_pub = this->create_publisher<freyja_msgs::msg::CurrentState>(
			current_state_topic, 10);
	}

private:
	char* frame_id;

	void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
	
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub;
	rclcpp::Publisher<freyja_msgs::msg::CurrentState>::SharedPtr current_state_pub;
};

void CurrentStatePub::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
	freyja_msgs::msg::CurrentState current_state{};
	
	current_state.header.frame_id = CurrentStatePub::frame_id;
	current_state.header.stamp.sec = this->get_clock()->now().nanoseconds() / pow(1000, 3);
	current_state.header.stamp.nanosec = this->get_clock()->now().nanoseconds();
	
	current_state.state_vector[0] = msg->position[0];	// pn
	current_state.state_vector[1] = msg->position[1];	// pe
	current_state.state_vector[2] = msg->position[2];	// pd

	current_state.state_vector[3] = msg->velocity[0];	// vn
	current_state.state_vector[4] = msg->velocity[1];	// ve
	current_state.state_vector[5] = msg->velocity[2];	// vd

	current_state.state_valid = true;
	
	current_state_pub->publish(current_state);
}