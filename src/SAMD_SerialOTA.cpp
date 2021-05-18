/*
  Copyright (c) 2017 Arduino LLC.  All right reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "SAMD_SerialOTA.h"

#include <SerialPackets.h>
#include <FlashStorage.h>

#include "SAMD_OTAInternalFlash.h"
#include <CRC32.h>



OTAInternalFlash my_internal_storage;

SerialPackets packetsOTA;
HardwareSerial* _hwSerial=nullptr;
#ifdef SOFTWARESERIAL
SoftwareSerial* _swSerial=nullptr;
#endif 
Stream* _debugPort=nullptr;

bool doLoop=true;
bool downloadingFirmware=false;
bool errorDownloadingFirmware=false;

// For CRC computation
CRC32 crc;


uint8_t flash_buff[OTAInternalFlash::BLOCK_SIZE]={0x00};
uint32_t flash_buff_size=0;
uint32_t flash_offset=0;
uint32_t firmware_size=0;


bool writeFirmwareBufferToFlash()
{
  if (errorDownloadingFirmware)
    return false;

  // Check if enough space to write the buffer
  if (flash_offset+OTAInternalFlash::BLOCK_SIZE<=my_internal_storage.get_flash_size())
  {
    uint8_t nbTrials=0;
    while(nbTrials<3)
    {
      // Write the buffer to the flash
      my_internal_storage.write(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE);
      // Compare the content of the flash with the one from the buffer
      if (my_internal_storage.sameContent(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE))
        break;
      nbTrials++;
      if (nbTrials>=3)
      {
        packetsOTA.printf("ERROR WRITING TO FLASH\n");
        errorDownloadingFirmware=true;
        return false;
      }
    }
  }
  else
  {
    // Send the message flash full
    packetsOTA.printf("ERROR NO SPACE LEFT\n");
    errorDownloadingFirmware=true;
    return false;
  }
  flash_offset=flash_offset+flash_buff_size;
  flash_buff_size=0;

  return true;
}

bool firmwarePacketBegin(uint8_t * payload, uint8_t payloadSize)
{
  // Init variables for uploading the firmware
  flash_buff_size=0;
  flash_offset=0;
  errorDownloadingFirmware=false;

  // If payload is defined, it should contain the FIRMWARE value
  if (payloadSize>0)
  {
    // Potential bug. SHould check the exact length
    if (!(payloadSize==strlen("FIRMWARE") && memcmp(payload,"FIRMWARE",strlen("FIRMWARE"))!=0))
    {
      // assume that payloadSize <=MAX_PAYLOAD_SIZE
      payload[payloadSize]=0x00;
      packetsOTA.printf("NOT A FIRMWARE \"%s\"",payload);
      errorDownloadingFirmware=true;
      return false;
    }
  }

  downloadingFirmware=true;
  return true;
}

bool firmwarePacketEnd(uint8_t * payload, uint8_t payloadSize)
{
  // Put back the led
  downloadingFirmware=false;
  // Write down the remaining data to the flash if any
  if (errorDownloadingFirmware)
    return false;
  bool OK=true;
  if (flash_buff_size>0)
    OK=writeFirmwareBufferToFlash();
  // Save the firmware size
  firmware_size=flash_offset;
  // Write the firmware
  if (OK)
  {
    packetsOTA.printf("WILL WRITE THE FIRMWARE\n");
    my_internal_storage.writeFirmwareAndReboot(firmware_size);
    // Will actually never reach this point
  }
  return OK;
}

//////////////////////////////////////////////////////////////////////////////
// Getting the firmware data from the serial and write it down to the flash //
//////////////////////////////////////////////////////////////////////////////
bool firmwarePacketReceived(uint8_t * payload, uint8_t payloadSize)
{
  if (errorDownloadingFirmware)
    return false;
  for(uint16_t i=0; i<payloadSize; i++)
  {
    // If the buffer is full
    if (flash_buff_size>=OTAInternalFlash::BLOCK_SIZE)
    {
      bool OK=writeFirmwareBufferToFlash();
      if (!OK)
        return false;
    }
    // Fill the buffer for the flash
    flash_buff[flash_buff_size]=payload[i];
    flash_buff_size++;
  }
  return true;
}

void scanFlashForErrors()
{
  packetsOTA.printf("START SCANNING FLASH\n");
  
  bool error=false;
  uint32_t flash_offset=0;

  // Generate values for the writing
  char c=0x30;
  for (uint16_t i=0;i<OTAInternalFlash::BLOCK_SIZE;i++)
  {
    flash_buff[i]=c;
    c++;
    if (c>0x39)
      c=0x30;
  }

  while(flash_offset<my_internal_storage.get_flash_size())
  {
    // Write the data to the flash
    my_internal_storage.write(flash_offset,(const void*)flash_buff,OTAInternalFlash::BLOCK_SIZE);
    
    // Compare the data
    if (!my_internal_storage.sameContent(flash_offset,(const void *)flash_buff,OTAInternalFlash::BLOCK_SIZE))
    {
      packetsOTA.printf("ERROR AT 0x%08X",my_internal_storage.get_flash_address()+flash_offset);
      error=true;
    }
    packetsOTA.printf("0x%08X",my_internal_storage.get_flash_address()+flash_offset);
    flash_offset=flash_offset+OTAInternalFlash::BLOCK_SIZE;
  }
  if (error==false)
    packetsOTA.printf("NO ERROR FOUND\n");
  else
    packetsOTA.printf("FINISHED SCANNING\n");  
}

// https://forum.arduino.cc/t/hex-string-to-byte-array/563827/3
byte nibble(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;  // Not a valid hexadecimal character
}

void hexCharacterStringToBytes(byte *byteArray, const char *hexString)
{
  bool oddLength = strlen(hexString) & 1;

  byte currentByte = 0;
  byte byteIndex = 0;

  for (byte charIndex = 0; charIndex < strlen(hexString); charIndex++)
  {
    bool oddCharIndex = charIndex & 1;

    if (oddLength)
    {
      // If the length is odd
      if (oddCharIndex)
      {
        // odd characters go in high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      }
      else
      {
        // Even characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    }
    else
    {
      // If the length is even
      if (!oddCharIndex)
      {
        // Odd characters go into the high nibble
        currentByte = nibble(hexString[charIndex]) << 4;
      }
      else
      {
        // Odd characters go into low nibble
        currentByte |= nibble(hexString[charIndex]);
        byteArray[byteIndex++] = currentByte;
        currentByte = 0;
      }
    }
  }
}


///////////////////////////////////////////////////////
// Read and print the flash at the specified address //
///////////////////////////////////////////////////////
void showFlashContent(uint32_t addr)
{
  uint32_t buff_offset=addr%OTAInternalFlash::BLOCK_SIZE;
  addr=addr-buff_offset;
  if (addr>=my_internal_storage.get_flash_size()+((uint32_t)my_internal_storage.get_flash_address()))
  {
    packetsOTA.printf("ADDR OUT OF RANGE\n");
    return;
  }
  my_internal_storage.readAtAbsoluteAddr(addr, flash_buff, OTAInternalFlash::BLOCK_SIZE);

  static char output[30];
  char *ptr = &output[0];
  uint8_t len=14;
  if (buff_offset+len>OTAInternalFlash::BLOCK_SIZE)
    len=OTAInternalFlash::BLOCK_SIZE-buff_offset;
  for (int i = 0; i < len; i++) {
    ptr += sprintf(ptr, "%02X", flash_buff[buff_offset+i]);
  }
  packetsOTA.printf("%s",output);
}


//////////////////////////////////////////////////
// Compute the CRC of the firmware in the flash //
//////////////////////////////////////////////////
uint32_t crcFlash() 
{
  CRC32 crc;
  uint32_t flash_offset=0;
  uint32_t buff_size=0;
  while(flash_offset<firmware_size)
  {
    buff_size=OTAInternalFlash::BLOCK_SIZE;
    if (buff_size+flash_offset>firmware_size)
      buff_size=firmware_size-flash_offset;
    my_internal_storage.read(flash_offset, flash_buff, buff_size);
    
    for (int i = 0; i < buff_size; i++) 
      crc.update(flash_buff[i]);
    flash_offset=flash_offset+buff_size;
  }
  return crc.finalize();
}

void packetReceived(uint8_t *payload, uint8_t payloadSize)
{
  char* cmd=(char*)payload;
  
  // Execute the commande
  if (strstr(cmd,"BAUD_")!=nullptr)
  {
    // Change the baud rate
    uint32_t bd=atoi(&cmd[strlen("BAUD_")]);
#ifdef SOFTWARESERIAL
  if (_hwSerial)
    _hwSerial->begin(bd);
  else
    _swSerial->begin(bd);
#else
    _hwSerial->begin(bd);
#endif
  }
  else if (strcmp(cmd,"FIRMWARE_CRC")==0)
  {
    // CRC32 from the flash
    uint32_t crc=crcFlash();
    packetsOTA.printf("%08X\n",crc);
  }
  else if (strstr(cmd,"PRINT_FLASH_")!=nullptr)
  {
    hexCharacterStringToBytes(flash_buff, &(cmd[strlen("PRINT_FLASH_")]));
    uint8_t tmp[4]={flash_buff[3],flash_buff[2],flash_buff[1],flash_buff[0]};
    uint32_t addr=((uint32_t*)tmp)[0];
    packetsOTA.printf("0x%08X\n",addr);
    showFlashContent(addr);
    return;
  }
  else if (strcmp(cmd,"SCAN_FLASH")==0)
  {
    scanFlashForErrors();
    return;
  }
  else if (strcmp(cmd,"FIRMWARE_SIZE")==0)
  {
    packetsOTA.printf("%d\n",firmware_size);
  }
  else if (strcmp(cmd,"BLOCK_SIZE")==0)
  {
    // CRC from the received serial data
    packetsOTA.printf("%d\n",OTAInternalFlash::BLOCK_SIZE);
  }
  else if (strcmp(cmd,"REBOOT")==0)
  {
    // Reboot the MCU
    NVIC_SystemReset();
    return;
  }
  else if (strcmp(cmd,"EXIT")==0)
  {
    // Continue to the main loop
    doLoop=false;
    packetsOTA.printf("EXITING OTA\n");
    return;
  }
  else if (strcmp(cmd,"SKETCH_ADDR")==0)
  {
    packetsOTA.printf("0x%08X\n",my_internal_storage.get_sketch_start_address());
  }
  else if (strcmp(cmd,"FLASH_ADDR")==0)
  {
    packetsOTA.printf("0x%08X\n",my_internal_storage.get_flash_address());
  }
  else if (strcmp(cmd,"FLASH_SIZE")==0)
  {
    packetsOTA.printf("%d\n",my_internal_storage.get_flash_size());
  }
  else if (strcmp(cmd,"WRITE_FIRMWARE")==0)
  {
    // The firmware should have been uploaded first (with no reboot in between)
    if (firmware_size==0)
      packetsOTA.printf("FIRMWARE NOT UPLOADED\n");
    else
      my_internal_storage.writeFirmwareAndReboot(firmware_size);
  }
  else if (strcmp(cmd,"VERSION")==0)
  {
    packetsOTA.printf("%s, %s\n", __DATE__, __TIME__);
  }
  else
  {
    // Send echo to the unknown cmd
    // cmd is supposed to be a readable string
    packetsOTA.printf("?%s\n",cmd);
  }
}



void packetError(uint8_t *payload, uint8_t payload_size)
{
  if (_debugPort)
    _debugPort->printf("samd: error sending packet\n");
}

void SAMD_SerialOTA::begin(HardwareSerial& hwSerial)
{
  _hwSerial=&hwSerial;
#ifdef SOFTWARESERIAL
  _swSerial=nullptr;
#endif
  packetsOTA.begin(*_hwSerial);
}

#ifdef SOFTWARESERIAL
void SAMD_SerialOTA::begin(SoftwareSerial& swSerial)
{
  _swSerial=&swSerial;
  _hwSerial=nullptr;
  packetsOTA.begin(*_swSerial);
}
#endif

void SAMD_SerialOTA::setDebugPort(Stream& stream)
{
  _debugPort=&stream;
}

void SAMD_SerialOTA::loop() 
{
  doLoop=true;
  pinMode(LED_BUILTIN, OUTPUT);

  packetsOTA.setReceiveCallback(packetReceived);
  packetsOTA.setErrorCallback(packetError);

  // For firmware upload
  packetsOTA.setOpenFileCallback(firmwarePacketBegin);
  packetsOTA.setReceiveFileDataCallback(firmwarePacketReceived);
  packetsOTA.setCloseFileCallback(firmwarePacketEnd);
  if (_debugPort)
    packetsOTA.setDebugPort(*_debugPort);

  packetsOTA.printf("ENTERING OTA\n");

  unsigned long prevTime=millis();
  while(doLoop)
  {
    uint16_t timer;
    if (downloadingFirmware)
      // Blink every 1/10 of second when receiving the firmware
      timer=100;
    else
      // Blink every second when idle to show that the MCU is alive and waiting for instructions through serial
      timer=1000;
    if (millis()-prevTime>timer)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      prevTime=millis();
    }
    // Process serial data
    packetsOTA.update();
    // To prevent triggering the WDT
    yield();
  }
}


