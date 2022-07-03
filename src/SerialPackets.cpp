#include "SerialPackets.h"



// The byte sequence for _packet_start_marker is different from any other values in the packet
// This is why the start marker is composed of 4 bytes.
// Any other bytes in the payload that are equal to one of  _packet_start_marker_ bytes are duplicated
// so that even if the payload contains the byte sequence of the start maker, this cannot be missinterpreted as start marker.
// See encodePayload() and decodePayload()

#define _packet_start_marker_1 0xFB
#define _packet_start_marker_2 0xFC
#define _packet_start_marker_3 0xFD
#define _packet_start_marker_4 0xFE

/* Usefull for debugging
#define _packet_start_marker_1 'S'
#define _packet_start_marker_2 'T'
#define _packet_start_marker_3 'A'
#define _packet_start_marker_4 'R'
*/

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(...)  do{ if (_debugPort) _debugPort->printf(__VA_ARGS__);} while(0)
#define DEBUG_PRINT_HEX(...)  do{ if (_debugPort) DebugPrintHex(_debugPort,__VA_ARGS__);} while(0)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_HEX(...)
#endif

void DebugPrintHex(Stream* _debugPort, const uint8_t *s, uint8_t len)
{
  int i=0;
  for (i = 0; i < len - 1; i++) 
    _debugPort->printf("%02X", s[i]);
  if (i < len)
    _debugPort->printf("%02X", s[i]);
}

SerialPackets::SerialPackets()
{

}

SerialPackets::~SerialPackets()
{

}

void SerialPackets::setACKTimeOut(uint16_t val) 
{
  _time_out=val;
}

uint16_t SerialPackets::getACKTimeOut() const 
{
  return _time_out;
}

// Get/set the number of retrials before calling the error callback and giving up
void SerialPackets::setNumberOfTrials(uint16_t val)
{
  _max_nb_trials=val;
}

uint16_t SerialPackets::getNumberOfTrials() const
{
  return _max_nb_trials;
}

void SerialPackets::setDebugPort(Stream& debugPort)
{
  _debugPort=&debugPort;
#ifndef DEBUG
  _debugPort->print("\"#define DEBUG\" should be added to the source file \"");
  _debugPort->print(__FILE__);
  _debugPort->println("\"");
#endif
}

void SerialPackets::begin(Stream& stream, uint8_t packetCounterInit)
{
  _stream=&stream;
  // Flush the stream for a clean start
  if (_stream)
      _stream->flush();
  // Reset all the variables
  _rx_prev_packet_counter=-1;
  _rx_prev_ack_packet_counter=-1;
  _rx_packet_counter=0;
  resetRx();
  memset(_rx_payload,0x00,sizeof(_rx_payload));

  // Data structure for sending the packets
  resetTx();
  // Ideally, _tx_packet_counter should be initialised with a random number
  _tx_packet_counter=packetCounterInit;
  _tx_ack_payload_size=0;
  _tx_ack_ready_to_be_resent=false;
}

void SerialPackets::end()
{
  _stream=nullptr;
}

uint16_t SerialPackets::checksum(uint8_t *buffer, uint8_t len)
{
  uint16_t c = 0;
  for (int i = 1; i < len; i++) {
    c += buffer[i];
  }
  return c;
}

void SerialPackets::setReceiveCallback(void (*callback)(uint8_t *,uint8_t))
{
  receiveDataCallback = callback;
}

void SerialPackets::setErrorCallback(void (*callback)(uint8_t *,uint8_t))
{
  errorNotificationCallback = callback;
}

// The payload should not contain any byte sequence which can be interpreted as the start marker
uint32_t SerialPackets::encodePayload(const uint8_t *payload, uint32_t &len, uint8_t *encodedPayload)
{
  uint32_t idx=0,encodeIdx=0;
  while(true)
  {
  	if (encodeIdx>=MAX_PAYLOAD_SIZE-1)
  	  break;
    if (idx>=len)
      break;

    if (payload[idx]==_packet_start_marker_1 ||
        payload[idx]==_packet_start_marker_2 ||
        payload[idx]==_packet_start_marker_3 ||
        payload[idx]==_packet_start_marker_4)
    {
      // If a _packet_start_marker_ is found in the payload
      // The byte is duplicated
      encodedPayload[encodeIdx]=payload[idx];
      encodedPayload[encodeIdx+1]=payload[idx];
      encodeIdx=encodeIdx+2;
    }
    else
    {
      encodedPayload[encodeIdx]=payload[idx];
      encodeIdx=encodeIdx+1;
    }
    idx++;
  }
  // Return the actual number of bytes that have been put to the payload
  len=idx;
  // Return the number of bytes that will be sent in the packet
  return encodeIdx;
}

uint32_t SerialPackets::decodePayload(uint8_t *payload, uint32_t len)
{

  uint32_t idx=1;
  while(true)
  {
    if (idx>=len)
      break;
      // If a _packet_start_marker_ byte is found in the payload
      // The next byte is removed
    if (payload[idx-1]==_packet_start_marker_1 ||
        payload[idx-1]==_packet_start_marker_2 ||
        payload[idx-1]==_packet_start_marker_3 ||
        payload[idx-1]==_packet_start_marker_4)
    {
      memcpy(&payload[idx],&payload[idx+1],sizeof(uint8_t)*(len-idx-1));
      len--;
    }
    idx++;
  }
  return len;
}

uint32_t SerialPackets::send(const uint8_t *payload, uint32_t len, bool blocking)
{
  if (!_stream)
  {
    DEBUG_PRINT("Serial port not defined\n");
    return 0;
  }

  //DEBUG_PRINT("Will send a packet of type %d\n",_tx_packet_type);

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
  _tx_payload_size=encodePayload(payload, len, _tx_payload);
  // Send the packet
  send_packet(_tx_payload, _tx_payload_size, _tx_packet_type, _tx_packet_counter);
  // Save the expected packet counter for the ACK
  _rx_ack_packet_counter=_tx_packet_counter;
  // Comes back to the default packet type
  _tx_status=TX_WAIT_ACK;
  _tx_time=millis();
  // If the function is blocking, we wait until the ACK packet is received or the timemout is over
  if (blocking)
    update(true);
  return len;
}


// Send a packet with payload and length. 
// Return the lenght that has been actually sent
void SerialPackets::send_packet(uint8_t *payload, uint8_t len, uint8_t packet_type, uint8_t packet_counter)
{
  //DEBUG_PRINT("Sending packet with type %d and size %d\n", packet_type, len);

  uint8_t tx_buffer[MAX_PAYLOAD_SIZE+10];
  uint8_t b = 0;

  // Start marker with four bytes
  // The purpose of using 4 bytes is to make the start marker different 
  // from any other byte sequence in the remaining bytes of the packet
  tx_buffer[b++] = _packet_start_marker_1;
  tx_buffer[b++] = _packet_start_marker_2;
  tx_buffer[b++] = _packet_start_marker_3;
  tx_buffer[b++] = _packet_start_marker_4;

  tx_buffer[b++] = packet_type;

  tx_buffer[b++] = packet_counter;

  tx_buffer[b++] = len;

  if (len>0) {
    memcpy(tx_buffer + b, payload, len);
  }
  b += len;

  uint16_t c = checksum(tx_buffer, b);
  tx_buffer[b++] = c >> 8; // checksum first byte (big/network endian)
  tx_buffer[b++] = c; // checksum second byte (big/network endian)

  // No end marker

  if (packet_type%2 == 1)
    DEBUG_PRINT("Sending ACK packet: ");
  else
    DEBUG_PRINT("Sending NORMAL packet: ");
  
  DEBUG_PRINT_HEX(tx_buffer, b);
  DEBUG_PRINT("\n");

  //if (_stream->availableForWrite()>b)  // This cause a bug for ESP8266
  _stream->write(tx_buffer, b);
}

void SerialPackets::processReceivedPacket()
{
  DEBUG_PRINT("Packet of type %d and counter 0x%02X arrived\n", _rx_packet_type, _rx_packet_counter);
  if (_rx_packet_type%2 == 0)
  {
    // If the packet type is DATA packet, we send the ACK packet
    // In case this is a new DATA packet (i.e. not a packet that is 
    // sent again because of not receiving the ACK packet)
    // If this ACK is new packet (i.e. not a packet that is sent again because of a lost packet)
    if (_rx_prev_packet_counter!=_rx_packet_counter)
    {
      _rx_prev_packet_counter=_rx_packet_counter;
      _tx_ack_ready_to_be_resent=false;
      // By default, no payload
      _tx_ack_payload_size=0;
      if (_rx_packet_type==PACKET_FILE_OPEN)
      {
        // Start receiving a file
        _rx_file_crc.reset();
        _rx_file_last_error=0;
        // If _rx_file_time!=0, this means that a file is already being sent 
        if (_rx_file_time!=0)
        {
          _rx_file_last_error=ERROR_FILE_ALREADY_OPENED;
          if (errorFileCallback!=nullptr)
            errorFileCallback(_rx_file_last_error);
        }
        else
        {
          // Call the callback
          if (openFileCallback!=nullptr)
          {
            memcpy(_rx_user_payload,_rx_payload,_rx_payload_size+1);
            _rx_user_payload_size=_rx_payload_size;
            // Call the callback
            bool OK=openFileCallback(_rx_user_payload,_rx_user_payload_size);
            _rx_user_payload_size=0;
            if (!OK)
              _rx_file_last_error=ERROR_FILE_USER;
          }
        }
        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          // If error, we send the error code
          sprintf((char*)_tx_ack_payload,"%d",_rx_file_last_error);
          _tx_ack_payload_size=3;
        }
        send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_resent=true;
        _rx_file_time=millis();
      }
      else if (_rx_packet_type==PACKET_FILE_DATA)
      {
        // If _rx_file_time==0, this means that the file has not been opened
        if (_rx_file_time==0)
        {
          _rx_file_last_error=ERROR_FILE_NOT_OPEN;
          if (errorFileCallback!=nullptr)
            errorFileCallback(_rx_file_last_error);
        }
        else
        {
          // Update the CRC32 for the received file
          for (int i = 0; i < _rx_payload_size; i++)
            _rx_file_crc.update(_rx_payload[i]);

          // Call the callback
          if (receiveFileDataCallback!=nullptr)
          {
	    memcpy(_rx_user_payload,_rx_payload,_rx_payload_size+1);
            _rx_user_payload_size=_rx_payload_size;
            bool OK=receiveFileDataCallback(_rx_user_payload,_rx_user_payload_size);
            _rx_user_payload_size=0;
            if (!OK)
              _rx_file_last_error=ERROR_FILE_USER;
          }
	}

        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          sprintf((char*)_tx_ack_payload,"%d",_rx_file_last_error);
          _tx_ack_payload_size=3;
        }
        send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_resent=true;

        _rx_file_time=millis();
      }
      else if (_rx_packet_type==PACKET_FILE_CLOSE)
      {
        // If _rx_file_time==0, this means that the file has not been opened
        if (_rx_file_time==0)
        {
          _rx_file_last_error=ERROR_FILE_NOT_OPEN;
          if (errorFileCallback!=nullptr)
            errorFileCallback(_rx_file_last_error);
        }
        else
        {
          // Finish receiving the file. Check matching CRC
          uint32_t rcvCrc;
          memcpy(&rcvCrc,_rx_payload,_rx_payload_size);
          // Check CRC of the received file
          uint32_t crc=_rx_file_crc.finalize();
          if (rcvCrc!=crc)
          {
            DEBUG_PRINT("CRC not matching for the uploaded file 0x%08X vs 0x%08X\n" ,crc, rcvCrc);
            // Send a message to indicate the mismatch of the CRC
            _rx_file_last_error=ERROR_FILE_WRONG_CRC;
            if (errorFileCallback!=nullptr)
              errorFileCallback(_rx_file_last_error);
          }
          else
          {
            DEBUG_PRINT("CRC matching for the uploaded file 0x%08X\n",crc);
            // The CRC match. We call the callback for closing the file
            if (closeFileCallback!=nullptr)
            {
              bool OK=closeFileCallback(rcvCrc);
              if (!OK)
                _rx_file_last_error=ERROR_FILE_USER;
            }
          }
        }
        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          sprintf((char*)_tx_ack_payload,"%d",_rx_file_last_error);
          _tx_ack_payload_size=3;
        }
        send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_resent=true;
        _rx_file_time=0;
      }
      else if (_rx_packet_type==PACKET_FILE_ABORT)
      {
        // File abord either by user or when the ACK is not received by the sender
        _rx_file_last_error=ERROR_FILE_ABORT;		// ERROR_FILE_ACK_NOT_RECEIVED
        if (errorFileCallback!=nullptr)
          errorFileCallback(_rx_file_last_error);
        send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_resent=true;
        _rx_file_time=0;
      }
      else if (_rx_packet_type==PACKET_DATA)  /*_tx_ack_packet_type==PACKET_DATA_ACK*/
      {
        // In this callback, the _tx_ack_payload is filled with the user's data
        if (receiveDataCallback!=nullptr)
        {
          memcpy(_rx_user_payload,_rx_payload,_rx_payload_size+1);
          _rx_user_payload_size=_rx_payload_size;
          // Send the ACK before the user's callback. This is to avoid delay due to the callback when sending the ACK packet
          send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
          _tx_ack_ready_to_be_resent=true;
          // Call the user callback
          receiveDataCallback(_rx_user_payload,_rx_user_payload_size);
          _rx_user_payload_size=0;
        }
        else
        {
          // No callback. The user is expected to use readString().
          memcpy(_rx_user_payload,_rx_payload,_rx_payload_size+1);
          _rx_user_payload_size=_rx_payload_size;
          _rx_user_payload_begin=0;

          // In fact, the ack should not be sent before readString()
          // Otherwise the next packet migh be sent immediatly and the current one migh be lost
          _tx_ack_ready_to_be_resent=false;
        }
      }
    }
    else
    {
      // The packet has been already received (duplicated packet because of the lost of the ACK packet)
      DEBUG_PRINT("Packet of type %d and counter 0x%02X already processed\n", _rx_packet_type, _rx_packet_counter);
      // The ACK is sent only when it is ready
      if (_tx_ack_ready_to_be_resent)
        send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
    }
  }
  else if (_rx_packet_type%2 == 1)
  {
    // _rx_packet_type%2==1 --> packet type is ACK
    //DEBUG_PRINT("ACK packet with type %d and counter 0x%02X received\n", _rx_packet_type, _rx_packet_counter);
    // ACK packet received. Now ready to send the next DATA packet
    if (_rx_prev_ack_packet_counter!=_rx_packet_counter)
    {
      _rx_prev_ack_packet_counter=_rx_packet_counter;
      // Process the payload from the ACK packet. The ACK packet is not supposed to contain user data
      if (_rx_packet_type==PACKET_FILE_OPEN_ACK || 
          _rx_packet_type==PACKET_FILE_DATA_ACK ||
          _rx_packet_type==PACKET_FILE_CLOSE_ACK )
      {
        // This indicates an error sent by the MCU receiving the file
        if (_rx_payload_size==3)
        {
          _tx_file_last_error=atoi((char*)_rx_payload);
        }
      }
      resetTx();
    }
    else
      DEBUG_PRINT("Packet with type %d and counter 0x%02X already processed\n", _rx_packet_type, _rx_prev_packet_counter);
  }
}

void SerialPackets::resetRx()
{
  _rx_status=RX_READY;
  _rx_idx=0;
  _rx_payload_size=0;
  _rx_packet_type=PACKET_UNDEFINED;
}

void SerialPackets::resetTx()
{
  _tx_packet_counter++;    // Increase the packet counter
  _tx_status=TX_READY;
  _tx_packet_type=PACKET_DATA;
  _tx_time=0;
  _tx_nb_trials=0;
  _tx_payload_size=0;
}

void SerialPackets::receive()
{
  //DEBUG_PRINT("SerialPackets::receive()\n");
  uint16_t counter=0;

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
      resetRx();
      continue;
    }

    _rx_status = RX_BUSY;
    _rx_buffer[_rx_idx] = b;

    if (_rx_idx == 0 && b != _packet_start_marker_1) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      resetRx();
      continue;
    }
    if (_rx_idx == 1 && b != _packet_start_marker_2) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      resetRx();
      continue;
    }
    if (_rx_idx == 2 && b != _packet_start_marker_3) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      resetRx();
      continue;
    }
    if (_rx_idx == 3 && b != _packet_start_marker_4) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      resetRx();
      continue;
    }
    if (_rx_idx == 4) 
    {
      // packet type: 
      _rx_packet_type = b;
      if (_rx_packet_type<PACKET_DATA || _rx_packet_type>PACKET_FILE_ABORT_ACK)
      {
          DEBUG_PRINT("Wrong packet type 0x%02X\n", _rx_packet_type);
          resetRx();
          continue;
      }
    }

    if (_rx_idx == 5) 
    {
      // Packet counter for the received packet
      _rx_packet_counter=b;
      // If we receive an ACK packet
      if (_rx_packet_type%2 == 1) 
      {
        if (_rx_packet_counter != _rx_ack_packet_counter)
        {
          // packet counter is same as previous tx packet
          DEBUG_PRINT("Wrong packet counter. Current value 0x%02X but should be 0x%02X\n", 
                      _rx_packet_counter, _rx_ack_packet_counter);
          resetRx();
          continue;
        }
      }
      else
      {
        // If we receive a normal packet, we prepare the packet type and counter for the ACK
        _tx_ack_packet_type=_rx_packet_type+1;
        _tx_ack_packet_counter=_rx_packet_counter;
      }
    }

    if (_rx_idx == 6) 
    {
      // payload size
      _rx_payload_size = b;
      if (_rx_payload_size > MAX_PAYLOAD_SIZE)
      {
        DEBUG_PRINT("Overflow with payload size %d\n", _rx_payload_size);
        resetRx();
        continue;
      }
    }

    if (_rx_idx == (6 + _rx_payload_size + 2))
    {
      // checksum
      uint16_t c1 = (_rx_buffer[_rx_idx - 1] << 8) + b;
      uint16_t c2 = checksum(_rx_buffer, _rx_idx -1);
      if (c1 != c2) 
      {
        DEBUG_PRINT("Wrong checksum for ");
        DEBUG_PRINT_HEX(_rx_buffer, _rx_idx);
        DEBUG_PRINT("\n");
        DEBUG_PRINT("Checksum is 0x%02X but should be 0x%02X\n",c1,c2);

        resetRx();
        continue;
      }
      // Copy the payload
      if (_rx_payload_size > 0)
      {
        memcpy(_rx_payload, &_rx_buffer[_rx_idx - _rx_payload_size - 1], _rx_payload_size);
      }
    }

    if (_rx_idx == (6 + _rx_payload_size + 2)) 
    {
      // Packet entirely received
      if (_rx_packet_type%2 == 0)
        DEBUG_PRINT("Received NORMAL packet of type %d, counter 0x%02X and payload ",_rx_packet_type,_rx_packet_counter);
      else
        DEBUG_PRINT("Received ACK packet of type %d, counter 0x%02X and payload ",_rx_packet_type,_rx_packet_counter);
      DEBUG_PRINT_HEX(_rx_payload, _rx_payload_size);
      DEBUG_PRINT("\n");
      _rx_idx = 0;
      // Decode the payload
      _rx_payload_size=decodePayload(_rx_payload, _rx_payload_size);
      // For conveniency for printing, add null char at the end of the payload
      // We assume _rx_payload_size<=MAX_PAYLOAD_SIZE)
      _rx_payload[_rx_payload_size]=0x00;
      // Process the packet which has just been received
      processReceivedPacket();
      // The payload is ready to be processed by the user
      _rx_status = RX_READY;
      // Should return here so that the packet can be received by the user
      return;
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

    // Check for the time out for receiving the file packets
    // After 5 seconds from the last file packet, we abord receiving the file
    if (_rx_file_time!=0 && (millis()-_rx_file_time > 5000))
    {
      _rx_file_time=0;
      _rx_file_last_error=ERROR_FILE_RECEIVING_TIMEOUT;
      if (errorFileCallback!=nullptr)
        errorFileCallback(_rx_file_last_error);
    }

    // Check for the time out for receiving the ACK packet
    if (_tx_status==TX_WAIT_ACK && (millis()-_tx_time > _time_out))
    {
      // Send the packet again
      if (_tx_nb_trials<_max_nb_trials)
      {
        DEBUG_PRINT("Retry sending %d of %d\n",_tx_nb_trials+1,_max_nb_trials);
        send_packet(_tx_payload, _tx_payload_size, _tx_packet_type, _tx_packet_counter);
        _tx_status=TX_WAIT_ACK;
        _tx_time=millis();
        _tx_nb_trials++;
        _nb_lost_packets++;
      }
      else
      {
        // This is to indicate the error to the user
        if (_tx_packet_type==PACKET_DATA)
        {
          // Send a notification with the payload that could not be sent
          if (errorNotificationCallback!=nullptr)
          {
            // Decode the payload before the callback
            _tx_payload_size=decodePayload(_tx_payload, _tx_payload_size);
            errorNotificationCallback(_tx_payload, _tx_payload_size);
          }
        }
        else
        {
          // In case of sending a file, we do not call the callback but set a flag
          _tx_file_last_error=ERROR_FILE_ACK_NOT_RECEIVED;
        }
        // Give up sending again the DATA package, call resetTx for sending next data
        resetTx();
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

// For using the SerialPackets as a stream. This is used indirectly by printf
size_t SerialPackets::write(uint8_t data)
{
  return send(&data, 1, true);
}

// For using the SerialPackets as a stream. This is used indirectly by printf
size_t SerialPackets::write(const uint8_t *buffer, size_t size)
{
  // If the size is too large, the buffer should be splitted to send several packets
  // This is to avoid cutting the strings
  size_t sentSize=0;
  while(sentSize<size)
    sentSize+=send(&buffer[sentSize], size-sentSize, true);
  return sentSize;
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
  // Process incoming packets
  update();
  // Return 0 if in the middle of receiving a packet
  if (_rx_status != RX_READY)
    return 0;
/*
  // Only data packet should be accessible through readString()
  if (_rx_packet_type!=PACKET_DATA)
    return 0;*/
  return _rx_user_payload_size-_rx_user_payload_begin;
}

// read() is used by readString()
int SerialPackets::read()
{
  // To avoid triggering the WDT
  yield();

  // Return -1 if in the middle of receiving a packet
  if (_rx_status != RX_READY)
    return -1;
/*
  // Only data packet should be accessible through readString()
  if (_rx_packet_type!=PACKET_DATA)
    return -1;*/
  if (_rx_user_payload_begin==_rx_user_payload_size)
    return -1;
  int val=_rx_user_payload[_rx_user_payload_begin];
  _rx_user_payload_begin++;
  if (_rx_user_payload_begin==_rx_user_payload_size)
  {
    _rx_user_payload_begin=0;
    _rx_user_payload_size=0;
    if (_tx_ack_ready_to_be_resent==false)
    {
      // The packet has been entirely read by the user (through readString).
      // It is time to send the ack in order to receive the next packet
      send_packet(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
      _tx_ack_ready_to_be_resent=true;
    }
  }
  return val;
}

int SerialPackets::peek() 
{
  // To avoid triggering the WDT
  yield();

  // Return -1 if in the middle of receiving a packet
  if (_rx_status != RX_READY)
    return -1;
/*
  // Only data packet should be accessible through readString()
  if (_rx_packet_type!=PACKET_DATA)
    return -1;*/
  if (_rx_user_payload_begin==_rx_user_payload_size)
    return -1;
  return _rx_user_payload[_rx_user_payload_begin];
}

// Start sending a file. 
int SerialPackets::openFile(const char* fileName)
{
  // Can send one file at a time
  _tx_file_last_error=0;

  // Wait for the previous ACK to be received
  update(true);

  // Reset the CRC32
  _tx_file_crc.reset();

  // Send the packet to intiate the file transfer
  _tx_packet_type=PACKET_FILE_OPEN;
  uint32_t lenSent;
  if (fileName!=nullptr)
    lenSent=send((const uint8_t*)fileName, strlen(fileName), false);
  else
    lenSent=send(nullptr, 0, false);
  if (_tx_file_last_error!=0)
    return _tx_file_last_error;
  else
    return lenSent;
}

// Send the file content
int SerialPackets::sendFileData(const uint8_t *payload, uint32_t len)
{
  // Feed the WDT
  yield();

  // Process the incoming packets
  update(true);

  if (_tx_file_last_error!=0)
    return _tx_file_last_error;
  if (_tx_status==TX_WAIT_ACK)
    return 0;
  // Send the packet
  _tx_packet_type=PACKET_FILE_DATA;
  uint32_t lenSent=send(payload, len, false);

  // Update CRC32
  for (int i = 0; i < lenSent; i++)
    _tx_file_crc.update(payload[i]);

  return lenSent;
}

int SerialPackets::closeFile()
{
  // Wait for the previous ACK to be received
  update(true);

  // If there was an error previously, we send an abord packet
  if (_tx_file_last_error==0)
    _tx_packet_type=PACKET_FILE_CLOSE;
  else
    _tx_packet_type=PACKET_FILE_ABORT;

  // Send the CRC32
  int32_t crc=_tx_file_crc.finalize();
  uint32_t lenSent=send((uint8_t*)&crc, sizeof(crc), false);

  // Wait for the ACK to be received
  update(true);

  return _tx_file_last_error;
}

int SerialPackets::abortFile()
{
  // Wait for the previous ACK to be received
  update(true);

  _tx_packet_type=PACKET_FILE_ABORT;

  // Send 0x00 for the CRC32
  int32_t crc=0;
  uint32_t lenSent=send((uint8_t*)&crc, sizeof(crc), false);

  // Wait for the previous ACK to be received
  update(true);

  return _tx_file_last_error;
}


void SerialPackets::setOpenFileCallback(bool (*callback)(uint8_t *,uint8_t))
{
  openFileCallback = callback;
}

void SerialPackets::setReceiveFileDataCallback(bool (*callback)(uint8_t *,uint8_t))
{
  receiveFileDataCallback = callback;
}

void SerialPackets::setCloseFileCallback(bool (*callback)(uint32_t))
{
  closeFileCallback = callback;
}

void SerialPackets::setErrorFileCallback(void (*callback)(int))
{
  errorFileCallback = callback;
}
