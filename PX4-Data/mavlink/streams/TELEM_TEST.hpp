// Test for sending MAVLink data received from a uORB topic to a MAVLink (SiK) telemetry radio

#include <uORB/topics/serial_control.h>

#ifndef TELEM_TEST_HPP
#define TELEM_TEST_HPP

class MavlinkStreamTelemTest : public MavlinkStream{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink){
        return new MavlinkStreamTelemTest(mavlink);
    }

    const char *get_name() const{
        return MavlinkStreamTelemTest::get_name_static();
    }

    static const char *get_name_static(){
        return "TELEM_TEST";
    }

    static uint16_t get_id_static(){
        return MAVLINK_MSG_ID_SERIAL_CONTROL;
    }

    uint16_t get_id(){
        return get_id_static();
    }

    unsigned get_size(){
        return MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _serial_rx_sub{ORB_ID::serial_rx};

protected:
    explicit MavlinkStreamTelemTest(Mavlink *mavlink) : MavlinkStream(mavlink){}

    bool send() override{
        serial_control_s serial_control_rx;
        _serial_rx_sub.copy(&serial_control_rx);

        mavlink_serial_control_t serial_control_tx;
        serial_control_tx.device = serial_control_rx.dev_id;
        serial_control_tx.flags = serial_control_rx.ctrl_flag;
        serial_control_tx.timeout = 0;
        serial_control_tx.baudrate = serial_control_rx.baud;
        serial_control_tx.count = serial_control_rx.count;
        serial_control_tx.target_system = 0;
        serial_control_tx.target_component = 0;

        for(uint8_t x = 0; x < serial_control_rx.DATA_SIZE; x++){
            serial_control_tx.data[x] = serial_control_rx.data[x];
        }

        mavlink_msg_serial_control_send_struct(_mavlink->get_channel(), &serial_control_tx);
        return true;
    }
};

#endif // TELEM_TEST_HPP