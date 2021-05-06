#ifndef SERIALPACKETS
#define SERIALPACKETS


#include "Arduino.h"



#define PACKET_UNDEFINED      0
#define PACKET_DATA           1
#define PACKET_ACK            2

#define TX_READY	      0
#define TX_WAIT_ACK           1
#define RX_READY	      0
#define RX_BUSY               1


class SerialPackets : public Stream
{
public:
    SerialPackets();
    ~SerialPackets();

    void begin(Stream& stream);
    void end();

    // Process the incoming packets and return the status. It can be TX_READY or TX_WAIT_ACK
    // If the parameter blocking is set to true, the function returns only when the next
    // packet can be sent straight away (i.e. the status is TX_READY)
    uint8_t update(bool blocking=false);

    // Send a packet. If the parameter blocking is set to true, the function only returns
    // when the ACK packet has been received or after the timeout (i.e. the status is TX_READY).
    // The next packet can then be sent straight away
    uint32_t send(const uint8_t *payload, uint32_t len, bool blocking=false);

    // Set the callback for receiving packets
    void setReceiveCallback(void (*callback)(uint8_t *,uint8_t)) __attribute__((always_inline))
    {
        receiveCallback = callback;
    }

    // Set the callback for error notification (ACK packet not received, etc) 
    void setErrorCallback(void (*callback)(uint8_t *,uint8_t)) __attribute__((always_inline))
    {
        errorNotificationCallback = callback;
    }

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
    void setTimeOut(uint16_t val) 
    {
      _time_out=val;
      setTimeout(_time_out*_max_nb_trials);
    }
    uint16_t getTimeOut() const { return _time_out; }

    // Get/set the number of retrials before calling the error callback and giving up
    void setNumberOfTrials(uint16_t val)
    {
      _max_nb_trials=val;
      setTimeout(_time_out*_max_nb_trials);
    }
    uint16_t getNumberOfTrials() const { return _max_nb_trials; }

    void setDebugPort(Stream& debugPort) { _debugPort=&debugPort; }

    // Maximum size of the payload for one packet
    static const uint8_t MAX_PAYLOAD_SIZE=30;

private:
    void send(uint8_t *payload, uint8_t len, uint8_t packet_type, uint8_t packet_counter);
    void receive();
    void processReceivedPacket();
private:
    uint16_t _time_out=100;
    uint16_t _max_nb_trials=2;

    void (*receiveCallback)(uint8_t *,uint8_t) = nullptr;
    void (*errorNotificationCallback)(uint8_t *,uint8_t) = nullptr;

    Stream* _stream=nullptr;
    Stream* _debugPort=nullptr;

    // Data structure for receiving and processing incoming packets
    uint8_t _rx_status=RX_READY;
    uint8_t _rx_idx=0;
    uint8_t _rx_buffer[MAX_PAYLOAD_SIZE+10];
    uint8_t _rx_payload[MAX_PAYLOAD_SIZE];
    uint8_t _rx_payload_size=0;
    uint8_t _rx_payload_begin=0;
    uint8_t _rx_packet_type=PACKET_UNDEFINED;
    uint8_t _rx_packet_counter=0;
    int _rx_prev_packet_counter=-1;

    // Data structure for sending the packets
    uint8_t _tx_status=TX_READY;
    unsigned long _tx_time=0;
    uint8_t _tx_nb_trials=0;
    uint8_t _tx_payload[MAX_PAYLOAD_SIZE];
    uint8_t _tx_payload_size=0;
    uint8_t _tx_ack_payload[MAX_PAYLOAD_SIZE];
    uint8_t _tx_ack_payload_size=0;
    uint8_t _tx_packet_counter=0;

    bool _ack_to_be_sent=false;
};


#endif    // SERIALPACKETS