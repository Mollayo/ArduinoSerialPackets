#include "SerialPackets.h"

#define _packet_start_marker 0x01
#define _packet_end_marker 0x04

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(...) if (_debugPort) { _debugPort->printf(__VA_ARGS__); }
#define DEBUG_PRINT_HEX(...) if (_debugPort) { DebugPrintHex(_debugPort,__VA_ARGS__); }
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_HEX(...)
#endif

void DebugPrintHex(Stream* _debugPort, const uint8_t *s, uint8_t len)
{
  int i=0;
  for (i = 0; i < len - 1; i++) 
    _debugPort->printf("%02X_", s[i]);
  if (i < len)
    _debugPort->printf("%02X", s[i]);
}

SerialPackets::SerialPackets()
{

}

SerialPackets::~SerialPackets()
{

}

void SerialPackets::begin(Stream& stream)
{
  _stream=&stream;
  // Flush the stream for a clean start
  if (_stream)
      _stream->flush();
  // Reset all the variables
  _rx_status=TX_READY;
  _rx_packet_counter=0;
  _rx_idx=0;
  _rx_payload_size=0;
  _rx_payload_begin=0;
  _rx_packet_type=PACKET_UNDEFINED;
  _rx_packet_counter=0;
  _rx_prev_packet_counter=-1;

  // Data structure for sending the packets
  _tx_status=TX_READY;
  _tx_time=0;
  _tx_nb_trials=0;
  _tx_payload_size=0;
  _tx_ack_payload_size=0;
  _tx_packet_counter=0;

  _ack_to_be_sent=false;

  setTimeout(_time_out*_max_nb_trials);
}

void SerialPackets::end()
{
    _stream=nullptr;
}

uint16_t crc(uint8_t *buffer, uint8_t len) {
  uint16_t c = 0;
  for (int i = 1; i < len; i++) {
    c += buffer[i];
  }
  return c;
}

uint32_t SerialPackets::send(const uint8_t *payload, uint32_t len, bool blocking)
{
  if (!_stream)
  {
    DEBUG_PRINT("Serial port not defined\n");
    return 0;
  }

  //In hardwareserial for uno sz==64 --> 31 is the max sz of tx pkt to be safe.
  // Overhead is 7 bytes
  if (len>MAX_PAYLOAD_SIZE)
    len=MAX_PAYLOAD_SIZE;

  if (_ack_to_be_sent)
  {
    // If the packet is sent while porcessing an ACK packet
    DEBUG_PRINT("\nACK load size %d\n",len);
    // Make a copy of the payload
    _tx_ack_payload_size=len;
    memcpy(_tx_ack_payload, payload, _tx_ack_payload_size);
    // Send the ACK packet
    send(_tx_ack_payload, _tx_ack_payload_size, PACKET_ACK, _rx_packet_counter);
    // To specify that the ACK packet has been sent
    _ack_to_be_sent=false;
    return len;
  }
  else
  {
    // If the function is blocking, we wait until the previous ACK packet is received or the timeout is over
    if (blocking)
      update(true);

    // Do not send the next packet if the ACK packet from the previous DATA packet has not been received
    if (_tx_status!=TX_READY)
    {
      DEBUG_PRINT("Cannot send packet because previous ACK packet not received\n");
      return 0;
    }
    // Copy the payload
    _tx_payload_size=len;
    memcpy(_tx_payload,payload,_tx_payload_size);
    // Send the packet
    send(_tx_payload, _tx_payload_size, PACKET_DATA, _tx_packet_counter);
    _tx_status=TX_WAIT_ACK;
    _tx_time=millis();
    // If the function is blocking, we wait until the ACK packet is received or the timemout is over
    if (blocking)
      update(true);
    return len;
  }
}


// Send a packet with payload and length. 
// Return the lenght that has been actually sent
void SerialPackets::send(uint8_t *payload, uint8_t len, uint8_t packet_type, uint8_t packet_counter)
{
  //DEBUG_PRINT("Sending packet with type %d and size %d\n", packet_type, len);

  uint8_t tx_buffer[MAX_PAYLOAD_SIZE+10];
  uint8_t b = 0;

  tx_buffer[b] = _packet_start_marker;
  b++;

  tx_buffer[b] = packet_counter;
  b++;

  tx_buffer[b++] = packet_type;
  tx_buffer[b++] = len;

  if (len>0) {
    memcpy(tx_buffer + b, payload, len);
  }
  b += len;

  uint16_t c = crc(tx_buffer, b);
  tx_buffer[b++] = c >> 8; // crc first byte (big/network endian)
  tx_buffer[b++] = c; // crc second byte (big/network endian)

  tx_buffer[b] = _packet_end_marker;
  b++;

  DEBUG_PRINT("Sending packet: ");
  DEBUG_PRINT_HEX(tx_buffer, b);
  DEBUG_PRINT("\n");

  _stream->write(tx_buffer, b);
}

void SerialPackets::processReceivedPacket()
{
  if (_rx_packet_type==PACKET_DATA)
  {
    // If the packet type is DATA packet, we send the ACK packet
    DEBUG_PRINT("MSG packet received\n");
    // In case this is a new DATA packet (i.e. not a packet that is 
    // sent again because of not receiving the ACK packet)
    _ack_to_be_sent=true;
    // If this ACK is new packet (i.e. not a packet that is sent again because of a lost packet)
    if (_rx_prev_packet_counter!=_rx_packet_counter)
    {
      // By default, the ACK packet has no payload
      _tx_ack_payload_size=0;
      if (receiveCallback!=nullptr)
        // In this callback, the _tx_ack_payload is filled with the user's data
        receiveCallback(_rx_payload,_rx_payload_size);
      _rx_prev_packet_counter=_rx_packet_counter;
    }
    // Send the ACK packet if not already done in the callback receiveCallback
    // For ACK packet, the packet counter is the one of the DATA packet that was previously received
    if (_ack_to_be_sent==true)
    {
      send(_tx_ack_payload, _tx_ack_payload_size, PACKET_ACK, _rx_packet_counter);
      _ack_to_be_sent=false;
    }
  }
  else if (_rx_packet_type==PACKET_ACK)
  {
    DEBUG_PRINT("ACK packet received\n");
    // ACK packet received. Now ready to send the next DATA packet
    _tx_packet_counter++;    // Increase the packet counter
    _tx_status=TX_READY;
    _tx_time=0;
    _tx_nb_trials=0;
    _tx_payload_size=0;
    _tx_nb_trials=0;
    // Callback for this ACK packet in case the user has put some data in it
    if (receiveCallback!=nullptr && _rx_payload_size>0)
        receiveCallback(_rx_payload,_rx_payload_size);
  }
  else
    DEBUG_PRINT("Unknown packet type\n");
}

void SerialPackets::receive()
{
  //DEBUG_PRINT("SerialPackets::receive()\n");
  uint8_t counter=0;

  while (_stream->available() > 0)
  {
    // This is to not being stucked in this wile loop if we constantly receive serial data
    counter++;
    if (counter>10)
      return;

    // To avoid trigerring the WDT
    yield();

    uint8_t b = _stream->read();
    //DEBUG_PRINT("_rx_idx: %d  byte: %02X\n", _rx_idx, b);

    if (_rx_idx == sizeof(_rx_buffer)) {
      DEBUG_PRINT("Rx buffer overflow\n");
      _rx_idx = 0;
      continue;
    }

    _rx_status = RX_BUSY;
    _rx_buffer[_rx_idx] = b;

    if (_rx_idx == 0 && b != _packet_start_marker) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      _rx_idx = 0;
      _rx_status = RX_READY;
      continue;
    }

    if (_rx_idx == 1) 
    {
      // Packet counter for the received packet
      _rx_packet_counter=b;
    }

    if (_rx_idx == 2) 
    {
      // packet type: 
      _rx_packet_type = b;
      // Check if the packet counter matches for ACK packet
      if (_rx_packet_type == PACKET_ACK) 
      {
        if (_rx_packet_counter != _tx_packet_counter)
        {
          // packet counter is same as previous tx packet
          DEBUG_PRINT("Wrong packet counter. Current value 0x%02X but should be 0x%02X\n", 
                      _rx_packet_counter, _tx_packet_counter);
          _rx_idx = 0;
          _rx_packet_type=PACKET_UNDEFINED;
          _rx_status = RX_READY;
          continue;
        }
      }
    }

    if (_rx_idx == 3) 
    {
      // payload size
      _rx_payload_size = b;
      _rx_payload_begin=0;
      if (_rx_payload_size > MAX_PAYLOAD_SIZE)
      {
        DEBUG_PRINT("Overflow with payload size %d\n", _rx_payload_size);
        _rx_idx = 0;
        _rx_packet_type = PACKET_UNDEFINED;
        _rx_payload_size = 0;
        _rx_status = RX_READY;
        continue;
      }
    }

    if (_rx_idx == (3 + _rx_payload_size + 2))
    {
      // checksum
      uint16_t c1 = (_rx_buffer[_rx_idx - 1] << 8) + b;
      uint16_t c2 = crc(_rx_buffer, _rx_idx -1);
      if (c1 != c2) 
      {
        DEBUG_PRINT("Wrong checksum for ");
        DEBUG_PRINT_HEX(_rx_buffer, _rx_idx);
        DEBUG_PRINT("\n");
        DEBUG_PRINT("Checksum is 0x%02X but should be 0x%02X\n",c1,c2);

        _rx_idx = 0;
        _rx_packet_type = PACKET_UNDEFINED;
        _rx_payload_size = 0;
        _rx_status = RX_READY;
        continue;
      }
      // Copy the payload
      if (_rx_payload_size > 0)
      {
        memcpy(_rx_payload, &_rx_buffer[_rx_idx - _rx_payload_size - 1], _rx_payload_size);
      }
    }

    if (_rx_idx == (3 + _rx_payload_size + 3) && b != _packet_end_marker) 
    { // end marker
      DEBUG_PRINT("Wrong end marker: 0x%02X\n", b);
      _rx_packet_type = PACKET_UNDEFINED;
      _rx_payload_size = 0;
      _rx_idx = 0;
      _rx_status = RX_READY;
      continue;
    }

    if (_rx_idx == (3 + _rx_payload_size + 3)) 
    {
      // end marker
      DEBUG_PRINT("Received packet with payload ");
      DEBUG_PRINT_HEX(_rx_payload, _rx_payload_size);
      DEBUG_PRINT("\n");
      // For conveniency for printing, add null char at the end of the payload
      if (_rx_payload_size<MAX_PAYLOAD_SIZE)
        _rx_payload[_rx_payload_size]=0x00;
      // Process the packet which has just been received
      processReceivedPacket();
      _rx_idx = 0;
      _rx_status = RX_READY;
      continue;
    }

    _rx_idx++;
  }
}

uint8_t SerialPackets::update(bool blocking)
{
  do
  {
    // Process incoming packets
    receive();

    // Check for the time out for receiving the ACK packet
    if (_tx_status==TX_WAIT_ACK && (millis()-_tx_time > _time_out))
    {
      // Send the packet again
      if (_tx_nb_trials<_max_nb_trials)
      {
        send(_tx_payload, _tx_payload_size, PACKET_DATA, _tx_packet_counter);
        _tx_status=TX_WAIT_ACK;
        _tx_time=millis();
        _tx_nb_trials++;
        DEBUG_PRINT("Retry sending %d of %d\n",_tx_nb_trials,_max_nb_trials);
      }
      else
      {
        // This is to indicate the error to the user
        if (errorNotificationCallback!=nullptr)
        {
          // Send a notification with the payload that could not be sent
          errorNotificationCallback(_tx_payload, _tx_payload_size);
        }
        // Give up sending again the DATA package
        _tx_status=TX_READY;
        _tx_time=0;
        _tx_nb_trials=0;
        _tx_payload_size=0;
        _tx_nb_trials=0;
      }
    }
    // To feed the watchdog timer
    yield();

    if (_tx_status==TX_READY)
      blocking=false;
  }
  while(blocking);

  return _tx_status;
}

// For using the SerialPackets as a stream
size_t SerialPackets::write(uint8_t data)
{
  return send(&data, 1, true);
}

size_t SerialPackets::write(const uint8_t *buffer, size_t size)
{
  return send(buffer, size, true);
}

int SerialPackets::availableForWrite()
{
  if (_tx_status==TX_READY)
    return MAX_PAYLOAD_SIZE-_tx_payload_size;
  else
    return 0;
}

int SerialPackets::available()
{
  if (_rx_status == RX_READY)
    return _rx_payload_size-_rx_payload_begin;
  else
    return 0;
}

int SerialPackets::read()
{
  // To avoid trigerring the WDT
  yield();

  // If not data, return -1
  if (_rx_status != RX_READY)
    return -1;
  if (_rx_payload_begin==_rx_payload_size)
    return -1;
  int val=_rx_payload[_rx_payload_begin];
  _rx_payload_begin++;
  if (_rx_payload_begin==_rx_payload_size)
  {
    _rx_payload_begin=0;
    _rx_payload_size=0;
  }
  return val;
}

int SerialPackets::peek() 
{
  // To avoid trigerring the WDT
  yield();

  if (_rx_status != RX_READY)
    return -1;
  if (_rx_payload_begin==_rx_payload_size)
    return -1;
  return _rx_payload[_rx_payload_begin];
}
