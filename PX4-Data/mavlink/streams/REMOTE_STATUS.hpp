#include "px4_platform_common/module_params.h"
#include <uORB/Subscription.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/battery_status.h>

#ifndef REMOTE_STATUS_HPP
#define REMOTE_STATUS_HPP

class MavlinkStreamRemoteStatus : public MavlinkStream, public ModuleParams {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)    { return new MavlinkStreamRemoteStatus(mavlink); }
    const char *get_name() const                            { return MavlinkStreamRemoteStatus::get_name_static(); }
    static const char *get_name_static()                    { return "REMOTE_STATUS"; }
    static uint16_t get_id_static()                         { return MAVLINK_MSG_ID_SERIAL_CONTROL; }
    uint16_t get_id()                                       { return get_id_static(); }
    unsigned get_size()                                     { return MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    void parameters_update();

    uORB::Subscription  _vehicle_status_sub{ORB_ID::vehicle_status};
    uORB::Subscription  _battery_status_sub{ORB_ID::battery_status};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    DEFINE_PARAMETERS(
        (ParamBool<px4::params::GNN_TELEM_ENABLE>)  _param_gnn_telem_enable,
        (ParamInt<px4::params::GNN_TELEM_PORT>)     _param_gnn_telem_port,
        (ParamInt<px4::params::GNN_TELEM_BAUD>)     _param_gnn_telem_baud,
        (ParamInt<px4::params::GNN_AGENT_ID>)       _param_gnn_agent_id
    )

protected:
    explicit MavlinkStreamRemoteStatus(Mavlink *mavlink) : MavlinkStream(mavlink), ModuleParams(nullptr){}

    bool send() override{

        parameters_update();

        if(_param_gnn_telem_enable.get()){

            vehicle_status_s vehicle_status;
            _vehicle_status_sub.copy(&vehicle_status);

            battery_status_s battery_status;
            _battery_status_sub.copy(&battery_status);

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
                buffer, "STAT: %d %d %d ",
                (int)_param_gnn_agent_id.get(),
                (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 2 : 1,
                (battery_status.warning > 0) ? 1 : 2
            );

            serial_control.count = fmin(70, len);
            serial_control.baudrate = _param_gnn_telem_baud.get();
            serial_control.flags = 0;
            serial_control.timeout = 0;
            serial_control.target_system = 0;
            serial_control.target_component = 0;
            for(int x = 0; x < serial_control.count; x++) serial_control.data[x] = buffer[x];

            mavlink_msg_serial_control_send_struct(_mavlink->get_channel(), &serial_control);
            return true;
        }

        return false;
    }

};

void MavlinkStreamRemoteStatus::parameters_update(){
    if(_parameter_update_sub.updated()){
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

#endif // REMOTE_STATUS_HPP