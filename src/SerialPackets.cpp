#include "SerialPackets.h"



// Ideally, the marker for _packet_start should be different from any other values in the packet
// The risk could be to start reading in the middle of the packets over and over
#define _packet_start_marker 0xFC
#define _packet_end_marker   0xFD

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
  _tx_ack_to_be_filled=false;
  _tx_ack_ready_to_be_sent=false;
}

void SerialPackets::end()
{
  _stream=nullptr;
}

uint16_t SerialPackets::crc(uint8_t *buffer, uint8_t len)
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

  if (_tx_ack_to_be_filled)
  {
    // If the packet is sent while processing an ACK packet
    // Make a copy of the payload
    _tx_ack_payload_size=len;
    memcpy(_tx_ack_payload, payload, _tx_ack_payload_size);
    // _tx_ack_packet_type is supposed to be PACKET_ACK, we send the ACK immediatly
    send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);

    _tx_ack_to_be_filled=false;
    return len;
  }
  else
  {
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
    _tx_payload_size=len;
    memcpy(_tx_payload,payload,_tx_payload_size);
    // Send the packet
    send(_tx_payload, _tx_payload_size, _tx_packet_type, _tx_packet_counter);
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
}


// Send a packet with payload and length. 
// Return the lenght that has been actually sent
void SerialPackets::send(uint8_t *payload, uint8_t len, uint8_t packet_type, uint8_t packet_counter)
{
  //DEBUG_PRINT("Sending packet with type %d and size %d\n", packet_type, len);

  uint8_t tx_buffer[MAX_PAYLOAD_SIZE+10];
  uint8_t b = 0;

  tx_buffer[b++] = _packet_start_marker;

  tx_buffer[b++] = packet_type;

  tx_buffer[b++] = packet_counter;

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

  if (packet_type%2 == 1)
    DEBUG_PRINT("Sending ACK packet: ");
  else
    DEBUG_PRINT("Sending NORMAL packet: ");
  
  DEBUG_PRINT_HEX(tx_buffer, b);
  DEBUG_PRINT("\n");

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
      _tx_ack_ready_to_be_sent=false;
      // By default, no payload
      _tx_ack_payload_size=0;
      if (_rx_packet_type==PACKET_FILE_OPEN)
      {
        // Start receiving a file
        _rx_file_crc.reset();
        _rx_file_last_error=0;

        // Call the callback
        if (openFileCallback!=nullptr)
        {
          memcpy(_rx_callback_payload,_rx_payload,_rx_payload_size+1);
          _rx_callback_payload_size=_rx_payload_size;
          bool OK=openFileCallback(_rx_callback_payload,_rx_callback_payload_size);
          if (!OK)
            _rx_file_last_error=ERROR_FILE_OTHER;
        }

        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          _tx_ack_payload[0]='E';	// E for Error
          _tx_ack_payload_size=1;
        }
        send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_sent=true;
      }
      else if (_rx_packet_type==PACKET_FILE_DATA)
      {
        // Call the callback
        if (receiveFileDataCallback!=nullptr)
        {
          memcpy(_rx_callback_payload,_rx_payload,_rx_payload_size+1);
          _rx_callback_payload_size=_rx_payload_size;
          bool OK=receiveFileDataCallback(_rx_callback_payload,_rx_callback_payload_size);
          if (!OK)
            _rx_file_last_error=ERROR_FILE_OTHER;
        }

        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          _tx_ack_payload[0]='E';	// E for Error
          _tx_ack_payload_size=1;
        }
        send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_sent=true;

        // Update the CRC32 for the received file
        for (int i = 0; i < _rx_payload_size; i++)
          _rx_file_crc.update(_rx_payload[i]);
      }
      else if (_rx_packet_type==PACKET_FILE_CLOSE)
      {
        // Finish receiving the file. Check matching CRC
        uint32_t rcvCrc;
        memcpy(&rcvCrc,_rx_payload,_rx_payload_size);
        uint32_t crc=_rx_file_crc.finalize();
        if (rcvCrc!=crc)
        {
          DEBUG_PRINT("CRC not matching for the uploaded file 0x%08X vs 0x%08X\n" ,crc, rcvCrc);
          // Send a message to indicate the mismatch of the CRC
          _rx_file_last_error=ERROR_FILE_OTHER;
        }
        if (rcvCrc==crc)
        {
          DEBUG_PRINT("CRC matching for the uploaded file 0x%08X\n",crc);
          // The CRC match. We call the callback for closing the file
          if (closeFileCallback!=nullptr)
            closeFileCallback((uint8_t*)&crc,sizeof(crc));
        }

        // Send the ACK
        if (_rx_file_last_error!=0)
        {
          _tx_ack_payload[0]='E';	// E for Error
          _tx_ack_payload_size=1;
        }
        send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
        _tx_ack_ready_to_be_sent=true;
      }
      else if (_tx_ack_packet_type==PACKET_DATA_ACK)
      {
        // In this callback, the _tx_ack_payload is filled with the user's data
        if (receiveDataCallback!=nullptr)
        {
          memcpy(_rx_callback_payload,_rx_payload,_rx_payload_size+1);
          _rx_callback_payload_size=_rx_payload_size;
          _tx_ack_to_be_filled=true;
          // Since the callback has been called, the payload should not be accessible through read() or readString()
          _rx_payload_size=0;
          _rx_payload_begin=0;
          // Call the user callback
          receiveDataCallback(_rx_callback_payload,_rx_callback_payload_size);
          // If the ACK not sent in the receiveDataCallback, we send it now
          if (_tx_ack_to_be_filled)
            send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
          _tx_ack_to_be_filled=false;
          _tx_ack_ready_to_be_sent=true;
        }
        else
        {
          // No callback. The user is expected to use readString().
          // In fact, the ack should not be sent before readString()
          // Otherwise the next packet migh be sent immediatly and the current one migh be lost
          _tx_ack_ready_to_be_sent=false;
        }
      }
    }
    else
    {
      DEBUG_PRINT("Packet of type %d and counter 0x%02X already processed\n", _rx_packet_type, _rx_packet_counter);
      // The ACK is sent only when it is ready
      if (_tx_ack_ready_to_be_sent)
        send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
    }
  }
  else if (_rx_packet_type%2 == 1)
  {
    //DEBUG_PRINT("ACK packet with type %d and counter 0x%02X received\n", _rx_packet_type, _rx_packet_counter);
    // ACK packet received. Now ready to send the next DATA packet
    if (_rx_prev_ack_packet_counter!=_rx_packet_counter)
    {
      _rx_prev_ack_packet_counter=_rx_packet_counter;
      resetTx();
      // Callback for this ACK packet in case the user has put some data in it
      if (_rx_packet_type==PACKET_DATA_ACK)
      {
        if (receiveDataCallback!=nullptr && _rx_payload_size>0)
        {
          memcpy(_rx_callback_payload,_rx_payload,_rx_payload_size+1);
          _rx_callback_payload_size=_rx_payload_size;
          receiveDataCallback(_rx_callback_payload,_rx_callback_payload_size);
        }
      }
      else if (_rx_packet_type==PACKET_FILE_OPEN_ACK || 
               _rx_packet_type==PACKET_FILE_DATA_ACK ||
               _rx_packet_type==PACKET_FILE_CLOSE_ACK )
      {
        // This indicates an error sent by the MCU receiving the file
        if (_rx_payload_size==1 && _rx_payload[0]=='E')
          _tx_file_last_error=ERROR_FILE_OTHER;
      }
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
  _rx_payload_begin=0;
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
      resetRx();
      continue;
    }

    _rx_status = RX_BUSY;
    _rx_buffer[_rx_idx] = b;

    if (_rx_idx == 0 && b != _packet_start_marker) 
    {
      // start marker
      DEBUG_PRINT("Received wrong start marker: 0x%02X\n", b);
      resetRx();
      continue;
    }

    if (_rx_idx == 1) 
    {
      // packet type: 
      _rx_packet_type = b;
      if (_rx_packet_type<PACKET_DATA || _rx_packet_type>PACKET_FILE_CLOSE_ACK)
      {
          DEBUG_PRINT("Wrong packet type 0x%02X\n", _rx_packet_type);
          resetRx();
          continue;
      }
    }

    if (_rx_idx == 2) 
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

    if (_rx_idx == 3) 
    {
      // payload size
      _rx_payload_size = b;
      _rx_payload_begin=0;
      if (_rx_payload_size > MAX_PAYLOAD_SIZE)
      {
        DEBUG_PRINT("Overflow with payload size %d\n", _rx_payload_size);
        resetRx();
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

        resetRx();
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
      resetRx();
      continue;
    }

    if (_rx_idx == (3 + _rx_payload_size + 3)) 
    {
      // end marker
      if (_rx_packet_type%2 == 0)
        DEBUG_PRINT("Received NORMAL packet of type %d, counter 0x%02X and payload ",_rx_packet_type,_rx_packet_counter);
      else
        DEBUG_PRINT("Received ACK packet of type %d, counter 0x%02X and payload ",_rx_packet_type,_rx_packet_counter);
      DEBUG_PRINT_HEX(_rx_payload, _rx_payload_size);
      DEBUG_PRINT("\n");
      _rx_idx = 0;
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

    // Check for the time out for receiving the ACK packet
    if (_tx_status==TX_WAIT_ACK && (millis()-_tx_time > _time_out))
    {
      // Send the packet again
      if (_tx_nb_trials<_max_nb_trials)
      {
        DEBUG_PRINT("Retry sending %d of %d\n",_tx_nb_trials+1,_max_nb_trials);
        send(_tx_payload, _tx_payload_size, _tx_packet_type, _tx_packet_counter);
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
            errorNotificationCallback(_tx_payload, _tx_payload_size);
        }
        else
        {
          // In case of sending a file, we do not call the callback but set a flag
          _tx_file_last_error=ERROR_FILE_ACK_NOT_RECEIVED;
        }
        // Give up sending again the DATA package, call initTx for sending next data
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
  if (_rx_status == RX_READY)
    return _rx_payload_size-_rx_payload_begin;
  else
    return 0;
}

// read() is used by readString()
int SerialPackets::read()
{
  // To avoid triggering the WDT
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
    if (_tx_ack_ready_to_be_sent==false)
    {
      // The packet has been entirely read by the user (through readString).
      // It is time to send the ack in order to receive the next packet
      send(_tx_ack_payload, _tx_ack_payload_size, _tx_ack_packet_type, _tx_ack_packet_counter);
      _tx_ack_ready_to_be_sent=true;
    }
  }
  return val;
}

int SerialPackets::peek() 
{
  // To avoid triggering the WDT
  yield();

  if (_rx_status != RX_READY)
    return -1;
  if (_rx_payload_begin==_rx_payload_size)
    return -1;
  return _rx_payload[_rx_payload_begin];
}

// Start sending a file. 
int SerialPackets::openFile(const char* fileName)
{
  // Can send one file at once
  _tx_file_last_error=0;

  // Wait for the previous ACK to be received
  update(true);

  // Reset the CRC32
  _tx_file_crc.reset();

  // Send the packet to intiate the file transfer
  _tx_packet_type=PACKET_FILE_OPEN;
  if (fileName!=nullptr)
    return send((const uint8_t*)fileName, strlen(fileName), false);
  else
    return send(nullptr, 0, false);
}

// Send the file content
int SerialPackets::sendFileData(const uint8_t *payload, uint32_t len)
{
  // Feed the WDT
  yield();

  // Process the incoming packets
  update(false);

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

int SerialPackets::closeFile(int32_t crc)
{
  // Wait for the previous ACK to be received
  update(true);

  _tx_packet_type=PACKET_FILE_CLOSE;

  // Send the CRC32
  if (crc==0)
    crc=_tx_file_crc.finalize();
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

void SerialPackets::setCloseFileCallback(void (*callback)(uint8_t *,uint8_t))
{
  closeFileCallback = callback;
}
