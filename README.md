# ArduinoSerialPackets
A library for sending/receiving packets over serial connection with CRC32 and ACK packets. One advantage of this library is the use of ACK packets and the ability to automatically resend lost data packets in case they have not been received. This makes the serial communication reliable even if some packets are lost. Connection speed with serial software can be set at 19200 bps and connection with hardware serial (on ESP8266) can be set at 3000000 bps.

Dependencies:
- CRC32 library: https://github.com/bakercp/CRC32
