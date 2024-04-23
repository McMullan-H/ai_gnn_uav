#include <uORB/topics/data_rx_test.h>
#include <uORB/topics/data_tx_test.h>

#ifndef DATA_TEST_OUTPUT_HPP
#define DATA_TEST_OUTPUT_HPP

class MavlinkStreamDataTestOutput : public MavlinkStream {
public:
    static MavlinkStream *new_instance(Mavlink *mavlink){
        return new MavlinkStreamDataTestOutput(mavlink);
    }

    const char *get_name() const{
        return MavlinkStreamDataTestOutput::get_name_static();
    }

    static const char *get_name_static(){
        return "DATA_TEST_OUTPUT";
    }

    static uint16_t get_id_static(){
        return MAVLINK_MSG_ID_DATA_TEST_OUTPUT;
    }

    uint16_t get_id(){
        return get_id_static();
    }

    unsigned get_size(){
        return MAVLINK_MSG_ID_DATA_TEST_OUTPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    uORB::Subscription _data_rx_test_sub{ORB_ID::data_rx_test};
    uORB::Publication<data_tx_test_s> _data_tx_test_pub{ORB_ID::data_tx_test};

protected:
    explicit MavlinkStreamDataTestOutput(Mavlink *mavlink) : MavlinkStream(mavlink){}

    bool send() override{
        data_rx_test_s data_rx;
        _data_rx_test_sub.copy(&data_rx);

        data_tx_test_s data_tx;
        data_tx.timestamp = hrt_absolute_time();

        if(data_rx.input_on){
            data_tx.output_on = true;
            data_tx.output_off = false;
        } else if(data_rx.input_off){
            data_tx.output_on = false;
            data_tx.output_off = true;
        }

        _data_tx_test_pub.publish(data_tx);

        mavlink_data_test_output_t dummy;

        mavlink_msg_data_test_output_send_struct(_mavlink->get_channel(), &dummy);
        return true;
    }
};

#endif // DATA_TEST_OUTPUT_HPP