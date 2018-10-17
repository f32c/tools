# f32c binary uploader (python and arduino)

Simple source code for uploading f32c binary executable over serial port.
(The port where "f32l>" prompt appears).

# Python

Python source to upload and execute f32c binary code.
Tested on linux.

TODO

    [x] High speed upload (3 Mbit)

# Arduino

Arduino source to upload and execute f32c binary code.
Tested on ESP32 arduino.

Needs f32c bistream with auto pass-thru option but
still it's not enough auto, serial break must be sent
from PC first then it will work:

    ujprog -r -P /dev/ttyUSB0

TODO

    [x] High speed upload (3 Mbit)
    [ ] How to send Serial break on ESP32
    [ ] Make f32c bitstream auto-switch to ESP32
        when serial break is detected on ESP32 line
