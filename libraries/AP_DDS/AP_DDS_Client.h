#pragma once

#if AP_DDS_ENABLED

#include "uxr/client/client.h"
#include "ucdr/microcdr.h"
#include "generated/Time.h"
#include "AP_DDS_Generic_Fn_T.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Scheduler.h>
#include <AP_HAL/Semaphores.h>
#include <AP_AHRS/AP_AHRS.h>

#include "fcntl.h"

#include <AP_Param/AP_Param.h>
#define STREAM_HISTORY 8
#define BUFFER_SIZE_SERIAL UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY

extern const AP_HAL::HAL& hal;

class AP_DDS_Client
{

private:

    // Serial Allocation
    uxrSerialTransport serial_transport; // client uxr serial transport
    uxrSession session; //Session

    // Input Stream
    uint8_t input_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_in;

    // Output Stream
    uint8_t output_reliable_stream[BUFFER_SIZE_SERIAL];
    uxrStreamId reliable_out;

    // Topic
    builtin_interfaces_msg_Time time_topic;

    // Data Writer
    const uxrObjectId dwriter_id = {
        .id = 0x01,
        .type = UXR_DATAWRITER_ID
    };

    HAL_Semaphore csem;

    // connection parametrics
    bool connected = true;

    static void update_topic(builtin_interfaces_msg_Time& msg);

public:
    // Constructor
    AP_DDS_Client();

    void main_loop(void);

    //! @brief Initialize the client's transport, uxr session, and IO stream(s)
    //! @return True on successful initialization, false on failure
    bool init() WARN_IF_UNUSED; 

    //! @brief Set up the client's participants, data read/writes,
    //         publishers, subscribers
    //! @return True on successful creation, false on failure
    bool create() WARN_IF_UNUSED;

    //! @brief Serialize the current data state and publish to to the IO stream(s)
    void write();
    //! @brief Update the internally stored DDS messages with latest data
    void update();

    //! @brief Parameter storage
    static const struct AP_Param::GroupInfo var_info[];

    //! @brief Convenience grouping for a single "channel" of data
    struct Topic_table {
        const uint8_t topic_id;
        const uint8_t pub_id;
        const char* topic_profile_label;
        const char* dw_profile_label;
        Generic_serialize_topic_fn_t serialize;
        Generic_deserialize_topic_fn_t deserialize;
        Generic_size_of_topic_fn_t size_of;
    };
    static const struct Topic_table topics[];


};

#endif // AP_DDS_ENABLED


