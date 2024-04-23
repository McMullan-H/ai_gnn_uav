#include "px4_platform_common/module_params.h"
#include <uORB/Subscription.hpp>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>

#ifndef GPS_RADIO_HPP
#define GPS_RADIO_HPP

class MavlinkStreamGPSRadio : public MavlinkStream, public ModuleParams {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)    { return new MavlinkStreamGPSRadio(mavlink); }
    const char *get_name() const                            { return MavlinkStreamGPSRadio::get_name_static(); }
    static const char *get_name_static()                    { return "GPS_RADIO"; }
    static uint16_t get_id_static()                         { return MAVLINK_MSG_ID_SERIAL_CONTROL; }
    uint16_t get_id()                                       { return get_id_static(); }
    unsigned get_size()                                     { return MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES; }

private:
    void parameters_update();

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::GNN_AGENT_ID>)        _param_gnn_agent_id,
        (ParamBool<px4::params::GNN_TELEM_ENABLE>)  _param_gnn_telem_enable,
        (ParamInt<px4::params::GNN_TELEM_PORT>)     _param_gnn_telem_port,
        (ParamInt<px4::params::GNN_TELEM_BAUD>)     _param_gnn_telem_baud
    )

    uORB::Subscription _vehicle_global_position_sub{ORB_ID::vehicle_global_position};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID::parameter_update, 1_s};

protected:
    explicit MavlinkStreamGPSRadio(Mavlink *mavlink) : MavlinkStream(mavlink), ModuleParams(nullptr) {}

    bool send() override{
        parameters_update();

        if(_param_gnn_telem_enable.get()){

            vehicle_global_position_s position;
            _vehicle_global_position_sub.copy(&position);

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
                buffer, "GPOS: %d %3.14f %3.14f %3.14f ",
                (int)_param_gnn_agent_id.get(),
                (double)position.lat,
                (double)position.lon,
                (double)position.alt
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

void MavlinkStreamGPSRadio::parameters_update(){
    if(_parameter_update_sub.updated()){
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();
    }
}

#endif // GPS_RADIO_HPP