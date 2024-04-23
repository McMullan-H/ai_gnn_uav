#include "px4_platform_common/module_params.h"
#include <uORB/Subscription.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>

#ifndef TRAJECTORY_SETPOINT_1_HPP
#define TRAJECTORY_SETPOINT_1_HPP

class MavlinkStreamTrajectorySetpoint1 : public MavlinkStream, public ModuleParams {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)    { return new MavlinkStreamTrajectorySetpoint1(mavlink); }
    const char *get_name() const                            { return MavlinkStreamTrajectorySetpoint1::get_name_static(); }
    static const char *get_name_static()                    { return "TRAJECTORY_SETPOINT_1"; }
    static uint16_t get_id_static()                         { return MAVLINK_MSG_ID_SERIAL_CONTROL; }
    uint16_t get_id()                                       { return get_id_static(); }
    unsigned get_size()                                     { return MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    void parameters_update();

    DEFINE_PARAMETERS(
        (ParamBool<px4::params::GNN_TELEM_ENABLE>) _param_gnn_telem_enable,
        (ParamInt<px4::params::GNN_TELEM_PORT>) _param_gnn_telem_port,
        (ParamInt<px4::params::GNN_TELEM_BAUD>) _param_gnn_telem_baud
    )

    uORB::Subscription _trajectory_setpoint_1_sub{ORB_ID::trajectory_setpoint_1};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

protected:
    explicit MavlinkStreamTrajectorySetpoint1(Mavlink *mavlink) : 
        MavlinkStream(mavlink),
        ModuleParams(nullptr)
        {}

    bool send() override{
        parameters_update();

        if(_param_gnn_telem_enable.get()){
            trajectory_setpoint_s setpoint;
            _trajectory_setpoint_1_sub.copy(&setpoint);

            mavlink_serial_control_t serial_control;
            
            switch(_param_gnn_telem_port.get()){
            case 1:
                serial_control.device = SERIAL_CONTROL_DEV_TELEM1;
                break;

            case 2:
                serial_control.device = SERIAL_CONTROL_DEV_TELEM2;
                break;

            default:
                serial_control.device = 255;
                break;
            }

            char buffer[70];
            uint8_t len = sprintf(
                buffer, "TRAJ: %d %3.5f %3.5f %3.5f ",
                1,
                (double)setpoint.velocity[0],
                (double)setpoint.velocity[1],
                (double)setpoint.velocity[2]
            );

            for(int x = 0; x < fmin(70, len); x++){
                serial_control.data[x] = buffer[x];
            }

            serial_control.count = fmin(70, len);
            serial_control.baudrate = _param_gnn_telem_baud.get();
            serial_control.flags = 0;
            serial_control.timeout = 0;
            serial_control.target_system = 0;
            serial_control.target_component = 0;

            mavlink_msg_serial_control_send_struct(_mavlink->get_channel(), &serial_control);
            return true;
        }

        return false;
    }
};

void MavlinkStreamTrajectorySetpoint1::parameters_update(){
    if(_parameter_update_sub.updated()){
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

#endif // TRAJECTORY_SETPOINT_1_HPP