//
// Created by Caden Keese on 3/6/22.
//

#ifndef CLUSTERDUCK_PROTOCOL_AGNODUCK_H
#define CLUSTERDUCK_PROTOCOL_AGNODUCK_H

#include <cassert>
#include <string>

#include <Arduino.h>
#include <WString.h>

enum muidStatus {
    invalid, // The MUID was not given in the correct format.
    unrecognized, // The MUID was not recognized. The Duck may have forgotten it.
    not_acked, // The MUID was recognized but not yet ack'd.
    acked // The MUID was recognized and has been ack'd.
};
#include "DuckCrypto.h"
#include "../DuckError.h"
#include "bloomfilter.h"
#include "cdpcfg.h"
#include "DuckPacket.h"
#include "DuckRadio.h"
#include "DuckTypes.h"
#include "DuckUtils.h"

class AgnoDuck {

public:
    /**
     * @brief Construct a new AgnoDuck object.
     *
     */
    AgnoDuck(String name="");

    virtual ~AgnoDuck();

    std::string getCDPVersion() { return duckutils::getCDPVersion(); }

    /**
     * @brief Set the Device Name object
     *
     * @param name
     */
    void setName(String name) { this->duckName = name; }

    /**
     * @brief Get the duck's name.
     *
     * @returns A string representing the duck's name
     */
    String getName() {return duckName;}
    /**
     * @brief setup the duck unique ID
     *
     * @param an 8 byte unique id
     * @return DUCK_ERR_NONE if successful, an error code otherwise
     */
    int setDeviceId(std::vector<byte> id);

    /**
     * @brief setup the duck unique ID
     *
     * @param an 8 byte unique id
     * @return DUCK_ERR_NONE if successful, an error code otherwise
     */
    int setDeviceId(byte* id);

    /**
     * @brief Setup serial connection.
     *
     * @param baudRate default: 115200
     */
    int setupSerial(int baudRate = 115200);

    /**
     * @brief Setup the radio component
     *
     * @param band      radio frequency in Mhz (default: 915.0)
     * @param ss        slave select pin (default CDPCFG_PIN_LORA_CS)
     * @param rst       reset pin  (default: CDPCFG_PIN_LORA_RST)
     * @param di0       dio0 interrupt pin (default: CDPCFG_PIN_LORA_DIO0)
     * @param di1       dio1 interrupt pin (default: CDPCFG_PIN_LORA_DIO1)
     * @param txPower   transmit power (default: CDPCFG_RF_LORA_TXPOW)
     */
    int setupRadio(float band = CDPCFG_RF_LORA_FREQ, int ss = CDPCFG_PIN_LORA_CS,
                   int rst = CDPCFG_PIN_LORA_RST, int di0 = CDPCFG_PIN_LORA_DIO0,
                   int di1 = CDPCFG_PIN_LORA_DIO1,
                   int txPower = CDPCFG_RF_LORA_TXPOW,
                   float bw = CDPCFG_RF_LORA_BW,
                   uint8_t sf = CDPCFG_RF_LORA_SF,
                   uint8_t gain = CDPCFG_RF_LORA_GAIN);

    /**
     * @brief Set sync word used to communicate between radios. 0x12 for private and 0x34 for public channels.
     *
     * @param syncWord set byte syncWord
     */
    void setSyncWord(byte syncWord);

    /**
     * @brief Set radio channel to transmit and receive on.
     *
     * @param channelNum set radio channel 1-5
     * @param isEU setSpectrum for EU use if needed
     */
    void setChannel(int channelNum, bool isEU);


    /**
     * @brief Sends data into the mesh network.
     *
     * @param topic the message topic
     * @param data a string representing the data
     * @param targetDevice the device UID to receive the message (default is no target device)
     * @param outgoingMuid Output parameter that returns the MUID of the sent packet. NULL is ignored.
     * @return DUCK_ERR_NONE if the data was send successfully, an error code otherwise.
     */
    int sendData(byte topic, const String data,
                 const std::vector<byte> targetDevice = ZERO_DUID, std::vector<byte> * outgoingMuid = NULL);

    /**
     * @brief Sends data into the mesh network.
     *
     * @param topic the message topic
     * @param data a vector of bytes representing the data to send
     * @param targetDevice the device UID to receive the message (default is no target device)
     * @param outgoingMuid Output parameter that returns the MUID of the sent packet. NULL is ignored.
     * @return DUCK_ERR_NONE if the data was send successfully, an error code
     otherwise.
     */
    int sendData(byte topic, std::vector<byte> bytes,
                 const std::vector<byte> targetDevice = ZERO_DUID, std::vector<byte> * outgoingMuid = NULL);

    /**
     * @brief Sends data into the mesh network.
     *
     * @param topic the message topic
     * @param data a string representing the data to send
     * @param targetDevice the device UID to receive the message (default is no target device)
     * @param outgoingMuid Output parameter that returns the MUID of the sent packet. NULL is ignored.
     * @return DUCK_ERR_NONE if the data was send successfully, an error code
     * otherwise.
     */
    int sendData(byte topic, const std::string data,
                 const std::vector<byte> targetDevice = ZERO_DUID, std::vector<byte> * outgoingMuid = NULL);

    /**
     * @brief Sends data into the mesh network.
     *
     * @param topic the message topic
     * @param data a byte buffer representing the data to send
     * @param length the length of the byte buffer
     * @param targetDevice the device UID to receive the message (default is no target device)
     * @param outgoingMuid Output parameter that returns the MUID of the sent packet. NULL is ignored.
     * @return DUCK_ERR_NONE if the data was send successfully, an error code
     * otherwise.
     */
    int sendData(byte topic, const byte* data, int length,
                 const std::vector<byte> targetDevice = ZERO_DUID, std::vector<byte> * outgoingMuid = NULL);

    /**
     * @brief Get the status of an MUID
     */
    muidStatus getMuidStatus(const std::vector<byte> & muid) const;

    /**
     * @brief Get an error code description.
     *
     * @param error an error code
     * @returns a string describing the error.
     */
    String getErrorString(int error);

    /**
     * @brief Turn on or off encryption.
     *
     * @param state true for on, false for off
     */
    void setEncrypt(bool state);

    /**
     * @brief get encryption state.
     *
     * @return true for on, false for off
     */
    bool getEncrypt();

    /**
     * @brief Turn on or off decryption. Used with MamaDuck
     *
     * @param state true for on, false for off
     */
    void setDecrypt(bool state);

    /**
     * @brief get decryption state.
     *
     * @return true for on, false for off
     */
    bool getDecrypt();

    /**
     * @brief Set new AES key for encryption.
     *
     * @param newKEY byte array, must be 32 bytes
     */
    void setAESKey(uint8_t newKEY[32]);

    /**
     * @brief Set new AES initialization vector.
     *
     * @param newIV byte array, must be 16 bytes
     */
    void setAESIv(uint8_t newIV[16]);

    /**
     * @brief Encrypt data using AES-256 CTR.
     *
     * @param text pointer to byte array of plaintext
     * @param encryptedData pointer to byte array to store encrypted message
     * @param inc size of text to be encrypted
     */
    void encrypt(uint8_t* text, uint8_t* encryptedData, size_t inc);

    /**
     * @brief Decrypt data using AES-256 CTR.
     *
     * @param encryptedData pointer to byte array to be decrypted
     * @param text pointer to byte array to store decrypted plaintext
     * @param inc size of text to be decrypted
     */
    void decrypt(uint8_t* encryptedData, uint8_t* text, size_t inc);

protected:
    AgnoDuck(AgnoDuck const&) = delete;
    AgnoDuck& operator=(AgnoDuck const&) = delete;

    String duckName="";

    String deviceId;
    std::vector<byte> duid;
    DuckRadio duckRadio;

    DuckPacket* txPacket = NULL;
    DuckPacket* rxPacket = NULL;
    std::vector<byte> lastMessageMuid;

    bool lastMessageAck = true;
    // Since this may be used to throttle outgoing packets, start out in a state
    // that indicates we're not waiting for a ack

    BloomFilter filter;

    /**
     * @brief sends a pong message
     *
     * @return DUCK_ERR_NONE if successfull. An error code otherwise
     */
    int sendPong();

    /**
     * @brief sends a ping message
     *
     * @return DUCK_ERR_NONE if successfull. An error code otherwise
     */
    int sendPing();

    /**
     * @brief Tell the duck radio to start receiving packets from the mesh network
     *
     * @return DUCK_ERR_NONE if successful, an error code otherwise
     */
    int startReceive();

    /**
     * @brief Implement the duck's specific behavior.
     *
     * This method must be implemented by the Duck's concrete classes such as DuckLink, MamaDuck,...
     */
    virtual void run() = 0;

    /**
     * @brief Setup a duck with default settings
     *
     * The default implementation simply initializes the serial interface.
     * It can be overriden by each concrete Duck class implementation.
     */
    virtual int setupWithDefaults(std::vector<byte> deviceId) {
        int err = setupSerial();
        if (err != DUCK_ERR_NONE) {
            return err;
        }
        err = setDeviceId(deviceId);
        if (err != DUCK_ERR_NONE) {
            return err;
        }
        return DUCK_ERR_NONE;
    }

    /**
     * @brief Get the duck type.
     *
     * @returns A value representing a DuckType
     */
    virtual int getType() = 0;


    /**
     * @brief Log an error message if the system's memory is too low.
     */
    static void logIfLowMemory();

    static bool imAlive(void*);
    static bool reboot(void*);
};

#endif //CLUSTERDUCK_PROTOCOL_AGNODUCK_H
