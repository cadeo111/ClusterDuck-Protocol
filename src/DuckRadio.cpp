#include "include/DuckRadio.h"

#if !defined(CDPCFG_HELTEC_CUBE_CELL)

#include "include/DuckUtils.h"

#define AM_PART_APOLLO3

#include <RadioLib.h>

#ifdef CDPCFG_PIN_LORA_SPI_SCK
#include "SPI.h"
SPIClass _spi;
SPISettings _spiSettings;
CDPCFG_LORA_CLASS lora =
    new Module(CDPCFG_PIN_LORA_CS, CDPCFG_PIN_LORA_DIO0, CDPCFG_PIN_LORA_RST,
               CDPCFG_PIN_LORA_DIO1, _spi, _spiSettings);
#else
CDPCFG_LORA_CLASS lora = new Module(CDPCFG_PIN_LORA_CS, CDPCFG_PIN_LORA_DIO0,
                                    CDPCFG_PIN_LORA_RST, CDPCFG_PIN_LORA_DIO1);
#endif

volatile uint16_t DuckRadio::interruptFlags = 0;
volatile bool DuckRadio::receivedFlag = false;

// if the radio is receiving a message
volatile bool radio_receiving = false;
// if the radio is sending a message
volatile bool radio_sending = false;
// if an interupt has happened
volatile bool interruptFired = false;

DuckRadio::DuckRadio() {}


int DuckRadio::setupRadio(LoraConfigParams config) {
    logwarn_f("~~ Selected Radio Frequency Band: %d\n", config.band);

#ifdef CDPCFG_SPARKFUN_APOLLO3
    lora = new Module(config.ss, config.di1, config.rst, config.di0, SPI1);
#elif CDPCFG_PIN_LORA_SPI_SCK
    log_n("_spi.begin(CDPCFG_PIN_LORA_SPI_SCK, CDPCFG_PIN_LORA_SPI_MISO, "
          "CDPCFG_PIN_LORA_SPI_MOSI, CDPCFG_PIN_LORA_CS)");
    _spi.begin(CDPCFG_PIN_LORA_SPI_SCK, CDPCFG_PIN_LORA_SPI_MISO,
               CDPCFG_PIN_LORA_SPI_MOSI, CDPCFG_PIN_LORA_CS);
    lora = new Module(config.ss, config.di0, config.rst, config.di1, _spi,
                      _spiSettings);
#else
    lora = new Module(config.ss, config.di0, config.rst, config.di1);
#endif

#ifdef CDPCFG_SPARKFUN_APOLLO3
    int rc = lora.begin(config.band, config.bw, config.sf, CDPCFG_SPARKFUN_APOLLO3_CODING_RATE,
                        CDPCFG_DEFAULT_SYNC_WORD,
                        config.txPower,
                        CDPCFG_SPARKFUN_APOLLO3_PREAMBLE_LENGTH,
                        CDPCFG_SPARKFUN_APOLLO3_TCXO_VOLTAGE,
                        CDPCFG_SPARKFUN_APOLLO3_USE_REGULATOR_LDO);
#elif
    int rc = lora.begin();
#endif
    if (rc != RADIOLIB_ERR_NONE) {
        logerr("ERROR  initializing LoRa driver. state = ");
        logerr(rc);
        return DUCKLORA_ERR_BEGIN;
    }
#ifndef CDPCFG_SPARKFUN_APOLLO3

    // Lora is started, we need to set all the radio parameters, before it can
    // start receiving packets
    rc = lora.setFrequency(config.band);
    if (rc == RADIOLIB_ERR_INVALID_FREQUENCY) {
        logerr("ERROR  frequency is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setBandwidth(config.bw);
    if (rc == RADIOLIB_ERR_INVALID_BANDWIDTH) {
        logerr("ERROR  bandwidth is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setSpreadingFactor(config.sf);
    if (rc == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
        logerr("ERROR  spreading factor is invalid");
        return DUCKLORA_ERR_SETUP;
    }

    rc = lora.setOutputPower(config.txPower);
    if (rc == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
        logerr("ERROR  output power is invalid");
        return DUCKLORA_ERR_SETUP;
    }


    rc = lora.setGain(config.gain);
    if (rc == RADIOLIB_ERR_INVALID_GAIN) {
        logerr("ERROR  gain is invalid");
        return DUCKLORA_ERR_SETUP;
    }

#endif
#ifdef CDPCFG_SPARKFUN_APOLLO3
    // set the interrupt handler to execute when packet tx or rx is done.
    lora.setDio1Action(config.func);

#else
    // set the interrupt handler to execute when packet tx or rx is done.
    lora.setDio0Action(config.func);
#endif
#ifndef CDPCFG_SPARKFUN_APOLLO3
    // set sync word to private network
    rc = lora.setSyncWord(CDPCFG_DEFAULT_SYNC_WORD);
    if (rc != RADIOLIB_ERR_NONE) {
        logerr("ERROR  sync word is invalid");
        return DUCKLORA_ERR_SETUP;
    }

#endif


    rc = lora.startReceive();

    if (rc != RADIOLIB_ERR_NONE) {
        logerr("ERROR Failed to start receive");
        return DUCKLORA_ERR_RECEIVE;
    }
    return DUCK_ERR_NONE;
}

void DuckRadio::setSyncWord(byte syncWord) {
    int error = lora.setSyncWord(syncWord);
    if (error != RADIOLIB_ERR_NONE) {
        logerr("ERROR  sync word is invalid");
    }
    lora.startReceive();
}

int DuckRadio::readReceivedData(std::vector<byte> *packetBytes) {

    int packet_length = 0;
    int err = DUCK_ERR_NONE;

    packet_length = lora.getPacketLength();

    if (packet_length < MIN_PACKET_LENGTH) {
        logerr("ERROR  handlePacket rx data size invalid: " +
               String(packet_length));

        DuckRadio::setReceiveFlag(false);
        int rxState = startReceive();
        return DUCKLORA_ERR_HANDLE_PACKET;
    }

    loginfo("readReceivedData() - packet length returns: " +
            String(packet_length));

    packetBytes->resize(packet_length);
    err = lora.readData(packetBytes->data(), packet_length);
    loginfo("readReceivedData() - lora.readData returns: " + String(err));

    DuckRadio::setReceiveFlag(false);
    int rxState = startReceive();

    if (err != RADIOLIB_ERR_NONE) {
        logerr("ERROR  readReceivedData failed. err: " + String(err));
        return DUCKLORA_ERR_HANDLE_PACKET;
    }

    loginfo("Rx packet: " + duckutils::convertToHex(packetBytes->data(), packetBytes->size()));
    loginfo("Rx packet: " + duckutils::toString(*packetBytes));

    loginfo("readReceivedData: checking path offset integrity");

    // Do some sanity checks on the received packet here before we continue
    // further RadioLib v4.0.5 has a bug where corrupt packets are still returned
    // to the app despite CRC check being enabled in the radio by both sender and
    // receiver.

    byte *data = packetBytes->data();

    loginfo("readReceivedData: checking data section CRC");

    std::vector<byte> data_section;
    data_section.insert(data_section.end(), &data[DATA_POS], &data[packet_length]);
    uint32_t packet_data_crc = duckutils::toUnit32(&data[DATA_CRC_POS]);
    uint32_t computed_data_crc =
            CRC32::calculate(data_section.data(), data_section.size());
    if (computed_data_crc != packet_data_crc) {
        logerr("ERROR data crc mismatch: received: " + String(packet_data_crc) +
               " calculated:" + String(computed_data_crc));
        return DUCKLORA_ERR_HANDLE_PACKET;
    }
    // we have a good packet
    loginfo("RX: rssi: " + String(lora.getRSSI()) +
            " snr: " + String(lora.getSNR()) +
#ifndef CDPCFG_SPARKFUN_APOLLO3
                    " fe: " + String(lora.getFrequencyError(true)) +
#endif
                    " size: " + String(packet_length));

    if (rxState != RADIOLIB_ERR_NONE) {
        return rxState;
    }

    return err;
}

int DuckRadio::sendData(byte *data, int length) {

    return startTransmitData(data, length);
}

int DuckRadio::relayPacket(DuckPacket *packet) {

    return startTransmitData(packet->getBuffer().data(),
                             packet->getBuffer().size());
}

int DuckRadio::sendData(std::vector<byte> data) {

    return startTransmitData(data.data(), data.size());
}

int DuckRadio::startReceive() {

    int state = lora.startReceive();
    radio_receiving = true;
    if (state != RADIOLIB_ERR_NONE) {
        logerr("ERROR startReceive failed, code " + String(state));
        return DUCKLORA_ERR_RECEIVE;
    }

    return DUCK_ERR_NONE;
}

int DuckRadio::getRSSI() { return lora.getRSSI(); }

// TODO: implement this
int DuckRadio::ping() { return DUCK_ERR_NOT_SUPPORTED; }

int DuckRadio::standBy() { return lora.standby(); }

int DuckRadio::sleep() { return lora.sleep(); }

void DuckRadio::processRadioIrq() {}

void DuckRadio::setChannel(int channelNum, bool isEU) {
    logdbg("Setting Channel to : ");
    logdbg(channelNum);

    int err;
    if (isEU) {
        switch (channelNum) {
            case 2:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_2_EU);
                lora.startReceive();
                break;
            case 3:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_3_EU);
                lora.startReceive();
                break;
            case 4:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_4_EU);
                lora.startReceive();
                break;
            case 5:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_5_EU);
                lora.startReceive();
                break;
            case 6:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_6_EU);
                lora.startReceive();
                break;
            case 1:
            default:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_1_EU);
                lora.startReceive();
                break;
        }
        switch (channelNum) {
            case 2:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_2);
                lora.startReceive();
                break;
            case 3:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_3);
                lora.startReceive();
                break;
            case 4:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_4);
                lora.startReceive();
                break;
            case 5:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_5);
                lora.startReceive();
                break;
            case 6:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_6);
                lora.startReceive();
                break;
            case 1:
            default:
                loginfo("Set channel: " + String(channelNum));
                err = lora.setFrequency(CHANNEL_1);
                lora.startReceive();
                break;
        }
    }
    if (err != RADIOLIB_ERR_NONE) {
        logerr("ERROR Failed to set channel");
    } else {
        lora.startReceive();
        channel = channelNum;
        loginfo("Channel Set");
    }
}

void DuckRadio::serviceInterruptFlags() {
#ifdef CDPCFG_SPARKFUN_APOLLO3
    if(interruptFired){
        if(radio_sending){
            loginfo("Interrupt was called while sending data, meaning data completely sent");
            radio_sending = false;
            startReceive();
        }else if(radio_receiving){
            loginfo("Interrupt was called while recieving data, meaning data received");
            DuckRadio::setReceiveFlag(true);
            radio_receiving = false;
        }
        // reset interrupt flag
        interruptFired = false;
    }
#else
    if (DuckRadio::interruptFlags != 0) {
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_TIMEOUT) {
            loginfo("Interrupt flag was set: timeout");
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE) {
            loginfo("Interrupt flag was set: packet reception complete");
            DuckRadio::setReceiveFlag(true);
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_PAYLOAD_CRC_ERROR) {
            loginfo("Interrupt flag was set: payload CRC error");
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_VALID_HEADER) {
            loginfo("Interrupt flag was set: valid header received");
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE) {
            loginfo("Interrupt flag was set: payload transmission complete");
            startReceive();
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_CAD_DONE) {
            loginfo("Interrupt flag was set: CAD complete");
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_FHSS_CHANGE_CHANNEL) {
            loginfo("Interrupt flag was set: FHSS change channel");
        }
        if (DuckRadio::interruptFlags & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_CAD_DETECTED) {
            loginfo("Interrupt flag was set: valid LoRa signal detected during CAD operation");
        }

        DuckRadio::interruptFlags = 0;
    }
#endif
}

// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
void DuckRadio::onInterrupt(void) {
#ifdef CDPCFG_SPARKFUN_APOLLO3
    interruptFired = true;
#else
    interruptFlags = lora.getIRQFlags();
#endif
}

int DuckRadio::startTransmitData(byte *data, int length) {
    int err = DUCK_ERR_NONE;
    int tx_err = RADIOLIB_ERR_NONE;
    loginfo("TX data");
    logdbg(" -> " + duckutils::convertToHex(data, length));
    logdbg(" -> length: " + String(length));
    radio_sending = true;
    long t1 = millis();
    // this is going to wait for transmission to complete or to timeout
    // when transmit is complete, the Di0 interrupt will be triggered
    tx_err = lora.transmit(data, length);
    switch (tx_err) {
        case RADIOLIB_ERR_NONE:

            loginfo("TX data done in : " + String((millis() - t1)) + "ms");
            break;

        case RADIOLIB_ERR_PACKET_TOO_LONG:
            // the supplied packet was longer than 256 bytes
            logerr("ERROR startTransmitData too long!");
            err = DUCKLORA_ERR_MSG_TOO_LARGE;
            break;

        case RADIOLIB_ERR_TX_TIMEOUT:
            logerr("ERROR startTransmitData timeout!");
            err = DUCKLORA_ERR_TIMEOUT;
            break;

        default:
            logerr("ERROR startTransmitData failed, err: " + String(tx_err));

            err = DUCKLORA_ERR_TRANSMIT;
            break;
    }

    return err;
}

#endif