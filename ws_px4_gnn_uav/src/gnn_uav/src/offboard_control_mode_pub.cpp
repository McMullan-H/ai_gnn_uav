#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class OffboardControlModePub : public rclcpp::Node {
public:
    bool is_armed = false;

    OffboardControlModePub() : Node("offboard_control_mode_pub") {

        RCLCPP_INFO(this->get_logger(), "Node: offboard_control_mode_pub started");

        int agent;

        this->declare_parameter("sim_en", 0);
        this->declare_parameter("agent", 0);

        this->get_parameter("sim_en", sim_en);
        this->get_parameter("agent", agent);

        char offboard_control_mode_topic[40], vehicle_command_topic[40], vehicle_status_topic[40];

        if(sim_en > 0){
            RCLCPP_WARN(this->get_logger(),
                "Warning: Simulation mode enabled. Additional ROS namespaces will be assigned to topics.");

            std::sprintf(offboard_control_mode_topic, "/px4_%d/fmu/in/offboard_control_mode", agent + 1);
            std::sprintf(vehicle_command_topic, "/px4_%d/fmu/in/vehicle_command", agent + 1);
            std::sprintf(vehicle_status_topic, "/px4_%d/fmu/out/vehicle_status", agent + 1);
        } else{
            std::sprintf(offboard_control_mode_topic, "/fmu/in/offboard_control_mode");
            std::sprintf(vehicle_command_topic, "/fmu/in/vehicle_command");
            std::sprintf(vehicle_status_topic, "/fmu/out/vehicle_status");
        }

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            offboard_control_mode_topic, 10);
        vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            vehicle_command_topic, 10);
        vehicle_status_sub = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            vehicle_status_topic, qos, std::bind(&OffboardControlModePub::vehicle_status_callback, this, _1));
        
        init_timer = this->create_wall_timer(100ms, std::bind(&OffboardControlModePub::init_timer_callback, this));
        arm_check_timer = this->create_wall_timer(3000ms, std::bind(&OffboardControlModePub::arm_check_timer_callback, this));
    }

private:
    int init_count = 0;
    bool is_init = false;
    int sim_en = 0;

    void init_timer_callback();
    void publish_offboard_control_mode();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg);
    void arm_check_timer_callback();
    void arm();
    void disarm();

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
    rclcpp::TimerBase::SharedPtr init_timer;
    rclcpp::TimerBase::SharedPtr arm_check_timer;
};

void OffboardControlModePub::init_timer_callback(){
        if(this->init_count == 10){

            // Change to offboard mode a second after startup
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            arm();

            is_init = true;
        }

        // Set to control flight velocity
        publish_offboard_control_mode();

        if(this->init_count <= 10) this->init_count++;
}

void OffboardControlModePub::publish_offboard_control_mode(){

    px4_msgs::msg::OffboardControlMode msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = true;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;

    offboard_control_mode_pub->publish(msg);
}

void OffboardControlModePub::publish_vehicle_command(uint16_t command, float param1, float param2){
    // Target system coincides with the Micro XRCE DDS client ID
    // 0 : Broadcast (client ID irrelevant), n > 0 : Target a specific client device (if sharing the same network)

    px4_msgs::msg::VehicleCommand msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 0;
    msg.target_component = 0;
    msg.source_system = 0;
    msg.source_component = 0;
    msg.from_external = true;

    vehicle_command_pub->publish(msg);
}

void OffboardControlModePub::vehicle_status_callback(const px4_msgs::msg::VehicleStatus::UniquePtr msg){
    switch(msg->arming_state){
    case px4_msgs::msg::VehicleStatus::ARMING_STATE_STANDBY:
        is_armed = false;
        break;
    
    case px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED:
        if(!is_armed) RCLCPP_INFO(this->get_logger(), "Setting state is_armed true");
        is_armed = true;
        break;

    default:
        is_armed = false;
        break;
    }
}

void OffboardControlModePub::arm_check_timer_callback(){
    // Send the arm command every 3 seconds if not already armed

    if(is_init && !is_armed){
        arm();
    }
}

void OffboardControlModePub::arm(){
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControlModePub::disarm(){
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

int main(int argc, char *argv[]){

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting offboard control mode publisher");

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControlModePub>());
	rclcpp::shutdown();
	return 0;
	
}