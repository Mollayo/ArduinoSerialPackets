# ArduinoSerialPackets
A library for sending/receiving packets over serial connection with CRC32 and ACK packets. One advantage of this library is the use of ACK packets and the ability to automatically resend lost data packets in case they have not been received. This makes the serial communication reliable even if some packets are lost. Connection speed with serial software can be set up to 19200 bps and connection with hardware serial (on ESP8266) can be set to 460800 bps. The file transfer rate is about 25 kB/s.

Dependencies:
- CRC32 library: https://github.com/bakercp/CRC32


