#ifndef SAMD_SERIAL_OTA_H_
#define SAMD_SERIAL_OTA_H_

#include <Arduino.h>

#if __has_include(<SoftwareSerial.h>)
# include <SoftwareSerial.h>
#define SOFTWARESERIAL
#endif

class SAMD_SerialOTA {
public:
  SAMD_SerialOTA() {}
  // Initialise with the serial connection, either HardwareSerial or SoftwareSerial
#ifdef SOFTWARESERIAL
  void begin(SoftwareSerial& swSerial);
#endif
  void begin(HardwareSerial& hwSerial);
  void setDebugPort(Stream& stream);
  // Enter the main loop. This should be placed in the setup or when the user wants to do the OTA update
  // This is a blocking function. In this loop, the MCU is listening to the serial connection waiting for
  // instructions for the OTA update
  void loop();
};

#endif        // SAMD_SERIAL_OTA_H_