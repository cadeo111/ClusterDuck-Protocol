//
// Created by Caden Keese on 3/6/22.
//

#ifndef CLUSTERDUCK_PROTOCOL_MAMADUCK_H
#define CLUSTERDUCK_PROTOCOL_MAMADUCK_H

#include <Arduino.h>
#include <WString.h>

#include "include/AgnoDuck.h"
#include "include/cdpcfg.h"
#include "include/DuckUtils.h"

class MamaDuck : public AgnoDuck {
public:
    using AgnoDuck::AgnoDuck;

    ~MamaDuck() {}

    using rxDoneCallback = void (*)(std::vector<byte> data );
    /**
     * @brief Register callback for handling data received from duck devices
     *
     * The callback will be invoked if the packet needs to be relayed (i.e not seen before)
     * @param cb a callback to handle data received by the papa duck
     */
    void onReceiveDuckData(rxDoneCallback cb) { this->recvDataCallback = cb; }

    /**
     * @brief Provide the DuckLink specific implementation of the base `run()`
     * method.
     *
     */
    void run();

    /**
     * @brief Override the default setup method to match MamaDuck specific
     * defaults.
     *
     * In addition to Serial component, the Radio component is also initialized.
     * When ssid and password are provided the duck will setup the wifi related
     * components.
     *
     * @param deviceId required device unique id
     * @param ssid wifi access point ssid (defaults to an empty string if not
     * provided)
     * @param password wifi password (defaults to an empty string if not provided)
     *
     * @returns DUCK_ERR_NONE if setup is successfull, an error code otherwise.
     */
    int setupWithDefaults(std::vector<byte> deviceId);

    /**
     * @brief Get the DuckType
     *
     * @returns the duck type defined as DuckType
     */
    int getType() {return DuckType::MAMA;}

    bool getDetectState();

private :
    rxDoneCallback recvDataCallback;
    void handleReceivedPacket();

    /**
     * @brief Handles if there were any acks addressed to this duck.
     *
     * @param packet The a broadcast ack, which has topic type reservedTopic::ack
     */
    void handleAck(const CdpPacket & packet);
};

#endif //CLUSTERDUCK_PROTOCOL_MAMADUCK_H
