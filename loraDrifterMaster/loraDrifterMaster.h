

// A. WiFi & Web Server
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// B. SPI & LoRa
#include <SPI.h>
#include <LoRa.h>
// B1. SPIFFS
#include "SPIFFS.h"

// C. GPS libraries 
#include <Wire.h>           // for reseting NMEA output from some T-Beam units
#include <TinyGPS++.h>      // decoding GPS 
#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_Ublox_GPS

// D. Power management on the TTGo T-beam
#include "axp20x.h"         // Not used on the versions I have but the new ones may need it

// E. Defines
#define nServantsMax  20           // Maximum number of servant drifters (just for setting array size)
#define nSamplesFileWrite  300      // Number of samples to store in memory before file write
#define RX_GPS  15                 // ESP32 onboard GPS pins
#define TX_GPS  12
#define SCK     5           // GPIO5  -- SX1278's SCK
#define MISO    19          // GPIO19 -- SX1278's MISO
#define MOSI    27          // GPIO27 -- SX1278's MOSI
#define SS      18          // GPIO18 -- SX1278's CS
#define RST     14          // GPIO14 -- SX1278's RESET
#define DI0     26          // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    915E6

// F. Function definitions
void resetGPSNMEAOutput(Stream &mySerial);
String processor(const String& var);
void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS);
void loraProcessRXData(int packetSize);




// G. Classes for Master and Servant Data

class Master
{
  public:
    Master();
    float lon;
    float lat;
    int hour;
    int minute;
    int second;
    int age;
};
Master::Master() {
  lon = 0.0;
  lat = 0.0;
  hour = 0;
  minute = 0;
  second = 0;
  age = 0;
};



class Servant
{
  public:
    Servant();
    void decode(String packet);
    void updateDistBear(float fromLon,float fromLat);
    int ID;
    int loraUpdatePlanSec;
    int lastUpdateMasterTime;
    int hour;
    int minute;
    int second;
    float lon;
    float lat;
    int age;
    int count;
    float dist;
    float bear;
    int rssi;
};
Servant::Servant() {
  ID = -1;
  loraUpdatePlanSec = 0;
  lastUpdateMasterTime = 0;
  hour = 0;
  minute = 0;
  second = 0;
  lon = 0.0;
  lat = 0.0;
  age = 0;
  count = 0;
  dist = 0.0;
  bear = 0.0;
  rssi = 0;
};

void Servant::updateDistBear(float fromLon,float fromLat) {
   dist=TinyGPSPlus::distanceBetween(fromLat, fromLon, lat, lon);
   bear=TinyGPSPlus::courseTo(fromLat, fromLon, lat, lon);
}

void Servant::decode(String packet) {
  // Example:  D01,15,6:36:15,-31.97758433,115.88428733,151,4104
  
  // Update Plan
  int comma1=packet.indexOf(",");
  int comma2=packet.indexOf(",",comma1+1);
  loraUpdatePlanSec=packet.substring(comma1+1,comma2).toInt();
  lastUpdateMasterTime=millis(); 
  // Time
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  String timeFull=packet.substring(comma1+1,comma2);
  
  int colon1=timeFull.indexOf(":");
  int colon2=timeFull.indexOf(":",colon1+1);
  hour=timeFull.substring(0,colon1).toInt();
  minute=timeFull.substring(colon1+1,colon2).toInt();
  second=timeFull.substring(colon2+1).toInt();
  
  // Latitude
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  lat=packet.substring(comma1+1,comma2).toFloat();
  
  // Longitude
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  lon=packet.substring(comma1+1,comma2).toFloat();

 // Age
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  age=packet.substring(comma1+1,comma2).toInt();

  // Count
  comma1=comma2;  comma2=packet.indexOf(",",comma1+1);
  count=packet.substring(comma1+1,comma2).toInt();
  
}



// H. This is the string literal for the main web page

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>UWA LoRa Drifters</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="1" >
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
    p {font-size: 3.0rem;}
    table, th, td { border: 1px solid black;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 6px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
  </style>
</head>
<body>

  <h2>LoRa Drifters</h2>
  
  <h4> Master Node </h4>
  <table>
  <tr><td>GPS Time</td>
  <td>Longitude</td>
  <td>Latitude</td>
  <td>GPS Age [milliSec]</td>
  <td>Get Data Link </td>
  <td>Last File Write GPS Time</td>
  <td>Erase Data (NO WARNING)</td>
  </tr>
  %MASTER%
  </table><br><br>
  
  <h4> Servants </h4>
  <table>
  <tr>
  <td>ID</td>
  <td>Lora Update Plan [sec]</td>
  <td>Last Update Master Time [sec] Ago</td>
  <td>Time</td>
  <td>Longitude</td>
  <td>Latitude</td>
  <td>Distance [m]</td>
  <td>Bearing [degN to]</td>
   <td>Count</td>
  <td>RSSI</td>
  </tr>
  %SERVANTS%
  </table>
  

</body>
</html>
)rawliteral";
