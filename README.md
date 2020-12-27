# Arduino with UART LoRa module as Simple APRS Tracker
This project is using a Arduino to decode GPS position information, and encode it into APRS packet, then send it over the air using UART LoRa module.

Only Arduino Mega 2560 or similar Arduino board with multiple hardware UART is supported.

# Operating Principle
It receives GPS data from hardware serial port 3 (Serial3), and sends non-standard AX.25 frame (checksum is omitted) to hardware serial port 1 (Serial1).

# Library Dependency
This project needs `TinyGPSPlus` to parse NMEA sentences.
