# loraTBeamOceanDrifter
Ocean drifters using LoRa radio with simple time slicing on a TTGO T-Beam

TTGO T-Beam modules are used for ocean drifter buoy tracking within the range of 
LoRa transmission. The same code can be used onshore as well, for example, for tracking
hikers in a region within LoRa range.

## Master Node
A Master node is used to receive LoRa radio packets from the Servant nodes. The
Master node activates an onboard WiFi Access point to present a WebServer to view real-time
data that is recevied by all of the Servant Nodes. The Master node also calculates
distance an bearing to all Servant Nodes real-time. 
The Master node records 1 second GPS positions of the Master node as well as any 
packets received from Servant Nodes.

## Servant Node
Each Servant node once turned on, records it's GPS location on the onboard PSRAM
and once per minute sends it's GPS location via LoRa radio.  GPS locations at a 1 
second interval are saved.

The servant node is configured with an ID and a time slice to make it's LoRa transmission.
The time slice is a nominated second of the minute at which the transmission will be
started.  GPS time is used for detection of the seconds at which to make the transmission.
In this way, all Servants that are receiving GPS time must be configured to use a
different second in the minute to make the LoRa transmission otherwise collision will occur.

The Servant node can be configured by activating the WebServer.  The button on the T-Beam is
used to activate and deactivate the WebServer on the Servant node.


# Libraries required:
For the Web Server:
https://github.com/me-no-dev/ESPAsyncWebServer
https://github.com/me-no-dev/AsyncTCP

For LoRa Communcations:
https://github.com/sandeepmistry/arduino-LoRa

For the GPS decoding:
https://github.com/mikalhart/TinyGPSPlus

For communicating with the u-blox to reset NMEA Serial output:
https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

Power management on the TTGO T-Beam:
axp20 code - need to track the source of this 
