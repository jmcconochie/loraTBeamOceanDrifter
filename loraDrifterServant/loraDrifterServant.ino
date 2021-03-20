
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
//#include "axp20x.h"         // Not used on the versions I have but the new ones may need it

// E. Defines

#define nSamplesFileWrite  300      // Number of samples to store in memory before file write
#define RX_GPS  15                 // ESP32 onboard GPS pins
#define TX_GPS  12
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  915E6


// =======================================================================================
// A. Global variables
// =======================================================================================
String drifterName = "D01";   // ID send with packet
int drifterTimeSlotSec = 15; // seconds after start of each GPS minute
TinyGPSPlus gps;
SFE_UBLOX_GPS myGPS;                  // U-blox object used for resetting the NMEA ouput
//AXP20X_Class axp;                     // power management chip on T-Beam
const char* ssid = "DrifterServant";   // Wifi ssid and password
const char* password = "Tracker1";
String csvOutStr = "";                // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);
bool webServerOn = false;
String csvFileName = "";
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
int ledState = LOW;
int ledPin = 14;
int webServerPin = 39;
int gpsLastSecond = -1;
int mhour, mminute, msecond;

// H. This is the string literal for the main web page

const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML><html>
  <head>
    <title>UWA LoRa Drifters</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <link rel="icon" href="data:,">
    <style>
      html {font-family: Arial; display: inline-block; text-align: center;}
      h2 {font-size: 3.0rem;}
      p {font-size: 3.0rem;}
      table, th, td { border: 1px solid black;}
      body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    </style>
  </head>
  <body>

    <h2>LoRa Drifters</h2>
    <h4> Servant Node </h4>
    <table>
    <td>Filename </td>
    <td>Get Data Link </td>
    <td>Last File Write GPS Time</td>
    <td>Erase Data (NO WARNING)</td>
    </tr>
    %SERVANT%
    </table><br><br>

    <h4> Configuration </h4>
    <form action="/configure" method="get">
    <table>
    <tr>
    <td> Setting </td>
    <td> Current Values </td>
    <td> New Values </td>
    <td> Guidance </td>
    </tr>
    
    <tr> 
    <td> <label for="fname">Drifter ID:</label> </td>
    <td> %DRIFTERID% </td>
    <td> <input type="text" id="fname" name="drifterID"></td>
    <td> Drifter IDs from D01 to D49 </td>
    </tr>
    <tr>
    <td> <label for="lname">LoRa Sending Second:</label> </td>
    <td> %LORASENDSEC% </td>
    <td> <input type="text" id="lname" name="loraSendSec"></td>
    <td>  Sending second is from 0 to 59 seconds </td>
    </tr>
    </table>
    <input type="submit" value="Configure">
    </form>

  </body>
  </html>
)rawliteral";

// =======================================================================================
// B. Setup
// =======================================================================================
void setup() {
  Serial.begin(115200);

  // A. TTGO Power ups
  //  Wire.begin(21, 22);
  //  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
  //    Serial.println("AXP192 Begin PASS");
  //  } else {
  //    Serial.println("AXP192 Begin FAIL");
  //  }
  //  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  //  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  //  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  //  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  //  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

  // B. Setup LEDs for information
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);     // will change state when a LoRa packet is received
  pinMode(webServerPin, INPUT);

  // C. Local GPS
  Serial2.begin(9600, SERIAL_8N1, TX_GPS, RX_GPS);   //Could be 15,12 or 34,12 or 35,12
  resetGPSNMEAOutput(Serial2);        // Will ensure that the GPS NMEA serial output is turned on

  //Setup LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.onTxDone(onTxDone);
  Serial.println("init ok");

  
  // G. SPIFFS to write data to onboard Flash
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
    while (1);
  }
  

  
  //H. Read config file if exists
  file = SPIFFS.open("/config.txt", FILE_READ);
  if (!file){
    Serial.println("Failed to open config.txt configuration file");
  } else {
    String inData=file.readStringUntil('\n');
    int comma = inData.indexOf(",");
    drifterName = inData.substring(0,comma);
    drifterTimeSlotSec = inData.substring(comma+1).toInt();
    Serial.println(inData);
    file.close();
  }

  csvFileName="/svt"+String(drifterName)+".csv";

  delay(1500);
}



// =======================================================================================
// C. Loop
// =======================================================================================
void loop() {
  if (!webServerOn) {


    // A. Receive and Encode GPS data
    unsigned long start = millis();
    do
    {
      while (Serial2.available())
        gps.encode(Serial2.read());
    } while (millis() - start < 500);
    
    // B. Send GPS data on LoRa if it is this units timeslot
    if (gps.time.second() == drifterTimeSlotSec) {
      Serial.println("sending packet");
      LoRa.beginPacket();
      String tTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
      String tLocation = String(gps.location.lat(), 8) + "," + String(gps.location.lng(), 8) + "," + String(gps.location.age());
      String sendPacket = String(drifterName) + "," + String(drifterTimeSlotSec) + "," + tTime + "," + tLocation + "," + String(nSamples) + "\n";
      LoRa.print(sendPacket);
      LoRa.endPacket(true);
      delay(1000); // Don't send more than 1 packet
      csvOutStr += sendPacket; // Save any packets that are sent (debugging purposes).
    }

    // C. If this is a new GPS record then save it
    if (gps.time.second() != gpsLastSecond) {
      float mlon = gps.location.lng();
      float mlat = gps.location.lat();
      mhour = gps.time.hour();
      mminute = gps.time.minute();
      msecond = gps.time.second();
      int mage = gps.location.age();
      gpsLastSecond = msecond;
      nSamples += 1;
      csvOutStr += String(mhour) +  "," + String(mminute) + "," + String(msecond) + "," + String(mlon, 8) + "," + String(mlat, 8) + "," + String(mage) + "\n";
    }

    // D. Write data to onboard flash if nSamples is large enough
    Serial.println(String(nSamples));
    if (nSamples > nSamplesFileWrite) {  // only write after collecting a good number of samples
      writeData2Flash();
    }
  }

  if (webServerOn){
    digitalWrite(ledPin, HIGH);
    delay(40);
    digitalWrite(ledPin, LOW); 
    delay(40);
  }

  // E. Check for button press
  if ( digitalRead(webServerPin) == LOW ) {
    if (webServerOn) {
      webServerOn = false;
      startWebServer(webServerOn);
      delay(1000);
    } else {
      webServerOn = true;
      startWebServer(webServerOn);
      delay(1000);
    }
  }


}



// =======================================================================================
// D. Functions
// =======================================================================================


// D0. Write data to flash
// 
void writeData2Flash (){
  file = SPIFFS.open(csvFileName, FILE_APPEND);
  if (!file) {
    Serial.println("There was an error opening the file for writing");
    lastFileWrite = "FAILED OPEN";
  } else {
    if (file.println(csvOutStr)) {
      file.close();
      csvOutStr = ""; nSamples = 0;
      lastFileWrite = String(mhour, DEC) + ":" + String(mminute, DEC) + ":" + String(msecond, DEC);
    } else {
      lastFileWrite = "FAILED WRITE";
    }
  }
}


// D1. LoRa has transmitted callback - flip flop the LED
void onTxDone() {
  Serial.println("TxDone");

  if (ledState == LOW) {
    digitalWrite(ledPin, HIGH);
    ledState = HIGH;
  } else {
    digitalWrite(ledPin, LOW);
    ledState = LOW;
  }
}


// D2. Reset NMEA serial output
//   Makes sure the onboard GPS has Serial NMEA output turned on

void resetGPSNMEAOutput(Stream &mySerial) {
  myGPS.begin(mySerial);
  myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}


// D3. Web Server Setup
void startWebServer(bool webServerOn) {

  if (webServerOn) {
    WiFi.softAP(ssid, password);
    Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

    // F. Web Server Callbacks setup
    server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
      request->send_P(200, "text/html", index_html, processor);
    });
    server.on("/configure", HTTP_GET,
      [](AsyncWebServerRequest * request) {
        int paramsNr = request->params();
        Serial.println(paramsNr);
        for(int i=0;i<paramsNr;i++){
            AsyncWebParameter* p = request->getParam(i);
            if (p->name() == "drifterID"){
              drifterName=p->value();
            }
            if (p->name() == "loraSendSec"){
              drifterTimeSlotSec=String(p->value()).toInt();
            }
        }
        csvFileName="/svt"+String(drifterName)+".csv";
        
        file = SPIFFS.open("/config.txt", FILE_WRITE);
        if (!file){
          Serial.println("Could not open config.txt for writing");
          request->send(200, "text/plain", "Failed writing configuration file config.txt!");
        } else {
          file.print(drifterName+","+String(drifterTimeSlotSec));
          file.close();
          request->send(200, "text/plain", "Success!");
        }
    });
    
    server.on("/getServant", HTTP_GET, [](AsyncWebServerRequest * request) {
      writeData2Flash();
      request->send(SPIFFS, csvFileName, "text/plain", true);
    });
    server.on("/deleteServant", HTTP_GET,
    [](AsyncWebServerRequest * request) {
      file = SPIFFS.open(csvFileName, FILE_WRITE);
      file.close();
      lastFileWrite = "";
      request->send(200, "text/plain", "Success!");
    });
    server.begin();
  } else {
    server.end();
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    btStop();
  }

}





// D4. Used to update sections of the webpages
//  Replaces placeholder with button section in your web page

String processor(const String& var) {
  if (var == "SERVANT") { 
    
     String servantData = "";
     servantData += "<td>"+csvFileName+"</td>";
     servantData += "<td><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"/getServant\"> GET </a></td>";
     servantData += "<td>" + lastFileWrite + "</td>";
     servantData += "<td><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"/deleteServant\"> ERASE </a></td>";
     servantData += "</tr>";
     return servantData;  
  }

  if (var == "DRIFTERID") {
    return drifterName;
  }
  if (var == "LORASENDSEC") {
    return String(drifterTimeSlotSec);
  }
  return String();
}


// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}
