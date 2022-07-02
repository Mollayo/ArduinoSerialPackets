
# ArduinoSerialPackets   [![Badge License]][License]

*Serial transmission library that includes **CRC32** & **ACK**.*

<br>

## Features

-   Reliable Serial Communication

-   Maximum Serial Speed
    
    - Software : `19200 bps`
    
    - Hardware : `460800 bps`

-   File Transfer Speed

    `~25 KB / s`
    
<br>
<br>

## Details

The library ensures reliable communication by <br>
resending lost ACK packages, as well as ones <br>
that have been damaged.

Damaged packages can be detected thought the <br>
use of the CRC32 cipher that acts as a checksum.

This managed transmission protocol can be especially <br>
useful when you want to send large amounts of data, <br>
such as firmware for instance.

<br>
<br>

## Dependencies

- **[CRC32]** by **[@bakercp]**

<br>


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-Unknown-darkgray.svg?style=for-the-badge

[@bakercp]: https://github.com/bakercp
[CRC32]: https://github.com/bakercp/CRC32

[License]: #
