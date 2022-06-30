#ifndef SERIALPACKETS
#define SERIALPACKETS


#include "Arduino.h"

// https://github.com/bakercp/CRC32
#include <CRC32.h>



#define PACKET_UNDEFINED      0xFF

#define PACKET_DATA           0x00
#define PACKET_DATA_ACK       0x01
#define PACKET_FILE_OPEN      0x02
#define PACKET_FILE_OPEN_ACK  0x03
#define PACKET_FILE_DATA      0x04
#define PACKET_FILE_DATA_ACK  0x05
#define PACKET_FILE_CLOSE     0x06
#define PACKET_FILE_CLOSE_ACK 0x07
#define PACKET_FILE_ABORT     0x08
#define PACKET_FILE_ABORT_ACK 0x09

#define TX_READY              0
#define TX_WAIT_ACK           1
#define RX_READY              0
#define RX_BUSY               1

#define ERROR_FILE_ACK_NOT_RECEIVED    -1
#define ERROR_FILE_WRONG_CRC           -2
#define ERROR_FILE_USER                -3
#define ERROR_FILE_ABORT               -4
#define ERROR_FILE_RECEIVING_TIMEOUT   -5
#define ERROR_FILE_NOT_OPEN            -6
#define ERROR_FILE_ALREADY_OPENED      -7

class SerialPackets : public Stream
{
public:
    SerialPackets();
    ~SerialPackets();

    void begin(Stream& stream, uint8_t packetCounterInit=0);
    void end();

    // Process the incoming packets and return the status. It can be TX_READY or TX_WAIT_ACK
    // If the parameter blocking is set to true, the function returns only when the next
    // packet can be sent straight away (i.e. the status is TX_READY)
    uint8_t update(bool blocking=false);

    // Send a packet. If the parameter blocking is set to true, the function only returns
    // when the ACK packet has been received or after the timeout (i.e. the status is TX_READY).
    // The next packet can then be sent straight away
    // The method returns the number of bytes that have been actually sent.
    // Be aware that this value can be different from the paramter "len"
    uint32_t send(const uint8_t *payload, uint32_t len, bool blocking=false);

    // Set the callback for receiving packets
    // The parameters of this callback are the payload of the packet and its length
    void setReceiveCallback(void (*callback)(uint8_t *,uint8_t));
    // Set the callback for error notification (ACK packet not received, etc)
    // The DATA packet that could not be sent are given as parameters to this callback
    void setErrorCallback(void (*callback)(uint8_t *,uint8_t));

    ////////////////////////////////
    // Methods for sending a file //
    ////////////////////////////////
    // Prepare for sending the file. Return 0 or error code
    int openFile(const char* fileName=nullptr);
    // Send file data. Return the number of bytes sent. This can be smaller than the parameter len.
    int sendFileData(const uint8_t *payload, uint32_t len);
    // Finish sending the file. This includes the CRC check
    int closeFile();
    // Abord sending the file.
    // The device which is receiving the file will receive an error code informing that the file tranfer has been aborted
    int abortFile();

    //////////////////////////////////
    // Methods for receiving a file //
    //////////////////////////////////
    // Callback for preparing receiving the file. The parameters are the file name and the length of the file name 
    void setOpenFileCallback(bool (*callback)(uint8_t *,uint8_t));
    // Callback for receiving the file data. The parameters are the data and its length
    void setReceiveFileDataCallback(bool (*callback)(uint8_t *,uint8_t));
    // Callback for finishing receiving the file data.
    void setCloseFileCallback(void (*callback)());
    // Error notification callback for receiving files
    // Might be called several times consecutively
    void setErrorFileCallback(void (*callback)(int));

    /////////////////////////////////////////
    // For using SerialPackets as a stream //
    /////////////////////////////////////////
    using Print::write; // Import other write() methods to support things like write(0) properly
    size_t write(uint8_t data);
    size_t write(const uint8_t *buffer, size_t size);
    int availableForWrite();
    int available();
    int read();
    int peek();
    void flush() {}


    // Get/set the time out in milliseconds for receiving the ACK packets
    void setACKTimeOut(uint16_t val);
    uint16_t getACKTimeOut() const;

    // Get/set the number of retrials before calling the error callback and giving up
    void setNumberOfTrials(uint16_t val);
    uint16_t getNumberOfTrials() const;

    void setDebugPort(Stream& debugPort);

    // Maximum size of the payload for one packet
    // Buffer size for software serial 64
    // Safe value: 30
    // https://arduino-esp8266.readthedocs.io/en/latest/reference.html#serial
    // Works 127, 200. Should not be more because the counter for the buzzer size is uint8_t
    static const uint8_t MAX_PAYLOAD_SIZE=200;

    // For statistics
    uint32_t _nb_lost_packets=0;
private:
    void send_packet(uint8_t *payload, uint8_t len, uint8_t packet_type, uint8_t packet_counter);
    void receive();
    void processReceivedPacket();
    void resetRx();
    void resetTx();
    uint16_t checksum(uint8_t *buffer, uint8_t len);
    uint32_t encodePayload(const uint8_t *payload, uint32_t &len, uint8_t *encodedPayload);
    uint32_t decodePayload(uint8_t *payload, uint32_t len);

private:
    uint16_t _time_out=50;        // Time after which the ACK packet is considered lost 
    uint16_t _max_nb_trials=40;   // 20 // Number of trials (resending the DATA packet) before giving up

    // Callback for error notification
    void (*errorNotificationCallback)(uint8_t *,uint8_t) = nullptr;

    // Callbacks for receiving data
    void (*receiveDataCallback)(uint8_t *,uint8_t) = nullptr;
    bool (*openFileCallback)(uint8_t *,uint8_t) = nullptr;
    bool (*receiveFileDataCallback)(uint8_t *,uint8_t) = nullptr;
    void (*closeFileCallback)() = nullptr;
    void (*errorFileCallback) (int) = nullptr;
    Stream* _stream=nullptr;
    Stream* _debugPort=nullptr;

    // Data structure for receiving and processing incoming packets
    uint8_t _rx_status=RX_READY;
    uint8_t _rx_idx=0;
    uint8_t _rx_buffer[MAX_PAYLOAD_SIZE+10];
    uint8_t _rx_payload[MAX_PAYLOAD_SIZE+1];
    uint8_t _rx_payload_size=0;
    uint8_t _rx_user_payload[MAX_PAYLOAD_SIZE+1];
    uint8_t _rx_user_payload_size=0;
    uint8_t _rx_user_payload_begin=0;
    uint8_t _rx_packet_type=PACKET_UNDEFINED;
    int _rx_packet_counter=0;
    int _rx_prev_packet_counter=-1;
    int _rx_prev_ack_packet_counter=-1;
    uint8_t _rx_ack_packet_counter=0;
    int8_t _rx_file_last_error=0;
    unsigned long _rx_file_time=0;
    CRC32 _rx_file_crc;

    // Data structure for sending the packets
    uint8_t _tx_status=TX_READY;
    unsigned long _tx_time=0;
    uint8_t _tx_nb_trials=0;
    uint8_t _tx_payload[MAX_PAYLOAD_SIZE+1];
    uint8_t _tx_payload_size=0;
    uint8_t _tx_packet_counter=0;
    uint8_t _tx_packet_type=PACKET_DATA;

    uint8_t _tx_ack_payload[MAX_PAYLOAD_SIZE+1];
    uint8_t _tx_ack_payload_size=0;
    uint8_t _tx_ack_packet_counter=0;
    uint8_t _tx_ack_packet_type=PACKET_UNDEFINED;
    bool _tx_ack_ready_to_be_resent=false;

    CRC32 _tx_file_crc;
    int8_t _tx_file_last_error=0;
};


#endif    // SERIALPACKETS
