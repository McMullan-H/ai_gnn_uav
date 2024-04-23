#include <rclcpp/rclcpp.hpp>
#include <freyja_msgs/msg/reference_state.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <gnn_uav_msgs/msg/reference_point.hpp>
#include <gnn_uav_msgs/msg/relative_position.hpp>

#include "gnn_math.hpp"

#include <chrono>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Capture Freyja reference state messages and translated them to PX4 trajectory setpoints

class TrajectorySetpointPub : public rclcpp::Node {
public:
	TrajectorySetpointPub() : Node("trajectory_setpoint_pub") {

		RCLCPP_INFO(this->get_logger(), "Node: trajectory_setpoint_pub started");

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		this->declare_parameter("sim_en", 0);
		this->declare_parameter("agent", 0);
		this->declare_parameter("hover_height", 0.0);
		this->declare_parameter("hover_yaw", 0.0);
		this->declare_parameter("hover_thresh", 0.0);
		this->declare_parameter("rise_rate", 0.0);
		this->declare_parameter("jitter_rate", 0.0);

		this->get_parameter("sim_en", sim_en);
		this->get_parameter("agent", agent);
		this->get_parameter("hover_height", hover_height);
		this->get_parameter("hover_yaw", hover_yaw);
		this->get_parameter("hover_thresh", hover_thresh);
		this->get_parameter("rise_rate", rise_rate);
		this->get_parameter("jitter_rate", jitter_rate);

		char param_initial_position[32];
		std::sprintf(param_initial_position, "uav_%d_initial_position", agent);

		this->declare_parameter(param_initial_position, std::vector<double>{});
		this->get_parameter(param_initial_position, initial_position);

		RCLCPP_INFO(this->get_logger(),
			"uav_%d: Initial position {%2.5f, %2.5f, %2.5f}",
			agent,
			initial_position[0],
			initial_position[1],
			initial_position[2]);

		char reference_state_topic[32];
		sprintf(reference_state_topic, "/uav_%d/reference_state", agent);

		char relative_position_topic[40];
		sprintf(relative_position_topic, "/uav_%d/relative_position", agent);

		char vehicle_global_position_topic[40];
		char trajectory_setpoint_topic[40];

		if(sim_en > 0){
			// Simulation conditions

			RCLCPP_WARN(this->get_logger(),
			"Warning: Simulation mode enabled. Additional ROS namespaces will be assigned to topics.");

			std::sprintf(vehicle_global_position_topic, "/px4_%d/fmu/out/vehicle_global_position", agent + 1);
			std::sprintf(trajectory_setpoint_topic, "/px4_%d/fmu/in/trajectory_setpoint", agent + 1);

		} else{
			// Real-world conditions

			if(agent > 0){
				// Topics to be broadcast through the telemetry radio
				std::sprintf(vehicle_global_position_topic, "/fmu/out/vehicle_global_position_%d", agent);
				std::sprintf(trajectory_setpoint_topic, "/fmu/in/trajectory_setpoint_%d", agent);
			} else{
				// Topics fixed to the first agent in the network (master)
				char str1[] = "/fmu/out/vehicle_global_position";
				char str2[] = "/fmu/in/trajectory_setpoint";
				memcpy(vehicle_global_position_topic, str1, sizeof(str1));
				memcpy(trajectory_setpoint_topic, str2, sizeof(str2));
			}
		}

		vehicle_global_position_sub = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
			vehicle_global_position_topic, qos, std::bind(&TrajectorySetpointPub::vehicle_global_position_callback, this, _1));
		
		reference_state_sub = this->create_subscription<freyja_msgs::msg::ReferenceState>(
			reference_state_topic, 10, std::bind(&TrajectorySetpointPub::reference_state_callback, this, _1));

		reference_point_sub = this->create_subscription<gnn_uav_msgs::msg::ReferencePoint>(
			"/reference_point", 10, std::bind(&TrajectorySetpointPub::reference_point_callback, this, _1));
			
		trajectory_setpoint_pub = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
			trajectory_setpoint_topic, 10);

		reference_point_pub = this->create_publisher<gnn_uav_msgs::msg::ReferencePoint>(
			"/reference_point", 10);

		relative_position_pub = this->create_publisher<gnn_uav_msgs::msg::RelativePosition>(
			relative_position_topic, 10);

		trajectory_setpoint_timer = this->create_wall_timer(
			100ms, std::bind(&TrajectorySetpointPub::trajectory_setpoint_timer_callback, this));

		reference_point_timer = this->create_wall_timer(
			1000ms, std::bind(&TrajectorySetpointPub::reference_point_timer_callback, this));

		setpoint_timeout = this->create_wall_timer(
			300ms, std::bind(&TrajectorySetpointPub::setpoint_timeout_callback, this));

	}

private:
	float		hover_height		= 0.0;
	float		hover_yaw			= 0.0;
	float		hover_thresh		= 0.0;
	float		rise_rate			= 0.0;
	float		jitter_rate			= 0.0;
	int			sim_en				= 0;
	int			agent				= 0;
	uint64_t	set_time_c			= 0;
	uint64_t	set_time_p			= 0;
	bool		at_hover_height		= false;
	bool 		init_coordinates	= false;
	bool		setpoint_valid		= false;

	// Initial NED frame displacement from reference point (meters)
	std::vector<double>	initial_position		= {0.0, 0.0, 0.0};

	// NED frame displacement relative to reference point (meters)
	std::vector<double> relative_position		= {0.0, 0.0, 0.0};

	// Initial coordinates (Lat/Lon/Alt)
	std::vector<double>	initial_coordinates		= {0.0, 0.0, 0.0};

	// Reference point location relative to first agent (Lat/Lon/Alt)
	std::vector<double> reference_coordinates	= {0.0, 0.0, 0.0};

	px4_msgs::msg::TrajectorySetpoint trajectory_setpoint{};
	px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_curr{};
	px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_prev{};

	void reference_state_callback(const freyja_msgs::msg::ReferenceState::UniquePtr msg);
	void vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
	void reference_point_callback(const gnn_uav_msgs::msg::ReferencePoint::UniquePtr msg);

	void trajectory_setpoint_timer_callback();
	void reference_point_timer_callback();
	void setpoint_timeout_callback();
	
	rclcpp::Subscription<freyja_msgs::msg::ReferenceState>::SharedPtr		reference_state_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr	vehicle_global_position_sub;
	rclcpp::Subscription<gnn_uav_msgs::msg::ReferencePoint>::SharedPtr		reference_point_sub;

	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr			trajectory_setpoint_pub;
	rclcpp::Publisher<gnn_uav_msgs::msg::ReferencePoint>::SharedPtr			reference_point_pub;
	rclcpp::Publisher<gnn_uav_msgs::msg::RelativePosition>::SharedPtr		relative_position_pub;

	rclcpp::TimerBase::SharedPtr	trajectory_setpoint_timer;
	rclcpp::TimerBase::SharedPtr	reference_point_timer;
	rclcpp::TimerBase::SharedPtr	setpoint_timeout;
};

void TrajectorySetpointPub::setpoint_timeout_callback(){
	// Prevents a sustained velocity command upon losing connection with the LQG controller node (safety function)

	if(	(trajectory_setpoint.velocity[0] == trajectory_setpoint_prev.velocity[0]) &&
		(trajectory_setpoint.velocity[1] == trajectory_setpoint_prev.velocity[1]) ){
		
		trajectory_setpoint.velocity[0] = 0;
		trajectory_setpoint.velocity[1] = 0;
	}

	trajectory_setpoint_prev.velocity[0] = trajectory_setpoint.velocity[0];
	trajectory_setpoint_prev.velocity[1] = trajectory_setpoint.velocity[1];
}

void TrajectorySetpointPub::reference_point_timer_callback(){
	// Broadcast reference point as a GPS setpoint
	// This is the location of the passage

	if((agent == 0) && init_coordinates){
		gnn_uav_msgs::msg::ReferencePoint reference;

		reference.lat = initial_coordinates[0] - M_TO_LAT(initial_position[0], 0, initial_coordinates[0]);
		reference.lon = initial_coordinates[1] - M_TO_LON(initial_position[1], 0, initial_coordinates[0]);
		reference.alt = initial_coordinates[2];

		reference_point_pub->publish(reference);
	}
}

void TrajectorySetpointPub::reference_point_callback(const gnn_uav_msgs::msg::ReferencePoint::UniquePtr msg){
	reference_coordinates[0] = msg->lat;
	reference_coordinates[1] = msg->lon;
	reference_coordinates[2] = msg->alt;
}

void TrajectorySetpointPub::reference_state_callback(const freyja_msgs::msg::ReferenceState::UniquePtr msg){

	if(at_hover_height && (agent == 0 || agent == 1 || agent == 2)){
		trajectory_setpoint.velocity[0] = msg->vn;
		trajectory_setpoint.velocity[1] = msg->ve;
	} else{
		trajectory_setpoint.velocity[0] = 0.0;
		trajectory_setpoint.velocity[1] = 0.0;
	}

}

void TrajectorySetpointPub::vehicle_global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg){
	
	if(!init_coordinates){

		// Initialize GPS offsets as starting coordinates
		initial_coordinates[0] = msg->lat;
		initial_coordinates[1] = msg->lon;
		initial_coordinates[2] = msg->alt;

		init_coordinates = true;

	} else{

		// Converting latitude/longitude/altitude to NED frame displacement relative to reference point

		gnn_uav_msgs::msg::RelativePosition position;

		double delta_lat = (msg->lat + reference_coordinates[0]) / 2;
		position.xn = LON_TO_M(msg->lon, reference_coordinates[1], delta_lat);
		position.ye = LAT_TO_M(msg->lat, reference_coordinates[0], delta_lat);
		position.zd = -(msg->alt - reference_coordinates[2]);

		relative_position[0] = position.xn;
		relative_position[1] = position.ye;
		relative_position[2] = position.zd;

		relative_position_pub->publish(position);
	}

}

void TrajectorySetpointPub::trajectory_setpoint_timer_callback(){
	// Remember that we are using NED frames

	trajectory_setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint.position = {NAN, NAN, NAN};
	trajectory_setpoint.yaw = hover_yaw;

	// Setting x-axis velocity as a cosine function of the hover height
	if((relative_position[2] != -hover_height) && (relative_position[2] > -(hover_height * 3))){
		trajectory_setpoint.velocity[2] = Z_VEL(relative_position[2], hover_height, rise_rate);
	} else{
		trajectory_setpoint.velocity[2] = 0;
	}

	// State to trigger the publishing of x and y velocity messages
	if((relative_position[2] <= -(hover_height - hover_thresh)) && (relative_position[2] >= -(hover_height + hover_thresh))){
		at_hover_height = true;
	} else{
		at_hover_height = false;
	}

	trajectory_setpoint_pub->publish(trajectory_setpoint);
}

int main(int argc, char *argv[]){

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting trajectory setpoint publisher");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectorySetpointPub>());
	rclcpp::shutdown();
	return 0;
	
}