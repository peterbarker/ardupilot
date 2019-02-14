#pragma once

#include <stdint.h>
#include <GCS_MAVLink/GCS.h>

class NodeStatusCb;

class AP_UAVCAN_MAVLinkBridge {
public:
    bool init(class AP_UAVCAN* ap_uavcan);

    ///// Configuration Parameters bridge /////
    static void handle_nodestatus(class AP_UAVCAN* ap_uavcan,
                                  uint8_t node_id,
                                  const NodeStatusCb &cb);
    void handle_message(const mavlink_message_t &msg);

};
