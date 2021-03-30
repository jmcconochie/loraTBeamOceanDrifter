
#include "loraDrifterMaster.h"

// =======================================================================================
// A. Global variables
// =======================================================================================
TinyGPSPlus gps;                      // decoder for GPS stream
SFE_UBLOX_GPS myGPS;                  // U-blox object used for resetting the NMEA ouput
AXP20X_Class axp;                     // power management chip on T-Beam
const char* ssid = "DrifterMaster";   // Wifi ssid and password
const char* password = "Tracker1";
Master m;                             // Master data
Servant s[nServantsMax];              // Servants data  array
String masterData = "";               // Strings for tabular data output to web page
String servantsData = "";
String csvOutStr = "";                // Buffer for output file
String lastFileWrite = "";
AsyncWebServer server(80);            // Create AsyncWebServer object on port 80
File file;                            // Data file for the SPIFFS output
int nSamples;                         // Counter for the number of samples gathered
int ledState = LOW;
int ledPin = 14;
int gpsLastSecond = -1;


// =======================================================================================
// B. Setup
// =======================================================================================

void setup() {
  Serial.begin(115200);

  // A. TTGO Power ups
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

  // B. Setup LEDs for information
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);     // will change state when a LoRa packet is received

  // C. Local GPS
  Serial2.begin(9600, SERIAL_8N1, TX_GPS, RX_GPS);   //Could be 15,12 or 34,12 or 35,12
  resetGPSNMEAOutput(Serial2);        // Will ensure that the GPS NMEA serial output is turned on

  // D. LoRa Setup
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.onReceive(loraProcessRXData);    // Callback to run when a packet is received


  // E. WiFi Access Point start up
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());    // Print ESP32 Local IP Address

  // F. Web Server Callbacks setup
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html, processor);
  });
  server.on("/getMaster", HTTP_GET, [](AsyncWebServerRequest * request) {
    writeData2Flash();
    request->send(SPIFFS, "/master.csv", "text/html", true);
  });
  server.on("/deleteMaster", HTTP_GET,
    [](AsyncWebServerRequest * request) {
      file = SPIFFS.open("/master.csv", FILE_WRITE);
      file.close();
      lastFileWrite="";
      request->send(200, "text/html", "<html><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"\">Success!  BACK </a></html>");
  });
  server.begin();

  // G. SPIFFS to write data to onboard Flash
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS - need to add retry");
    while (1);
  }

  // H. Start LoRa reception
  LoRa.receive();

}




// =======================================================================================
// C. Loop
// =======================================================================================
void loop() {
  // A. LoRa is on interrupt / callback

  // B. Read and decode Master GPS
  SerialGPSDecode(Serial2, gps);

  // C. Make Servants Data HTML
  servantsData = "";
  for (int i = 0; i < nServantsMax; i++) {
    String tTime = String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    servantsData += "<tr>";
    servantsData += "<td>" + String(s[i].ID) + "</td>";
    servantsData += "<td>" + String(s[i].loraUpdatePlanSec) + "</td>";
    servantsData += "<td>" + String((millis() - s[i].lastUpdateMasterTime) / 1000) + "</td>";
    servantsData += "<td>" + String(s[i].hour) + ":" + String(s[i].minute) + ":" + String(s[i].second) + "</td>";
    servantsData += "<td>" + String(s[i].lon, 8) + "</td>";
    servantsData += "<td>" + String(s[i].lat, 8) + "</td>";
    servantsData += "<td>" + String(s[i].dist) + "</td>";
    servantsData += "<td>" + String(s[i].bear) + "</td>";
    servantsData += "<td>" + String(s[i].count) + "</td>";
    servantsData += "<td>" + String(s[i].rssi) + "</td>";
    servantsData += "</tr>";
  }

  // D. Write data to onboard flash
  if (nSamples > nSamplesFileWrite) {  // only write after collecting a good number of samples
    writeData2Flash();
  }

  // Save to SD Card file ???


}


// =======================================================================================
// D. Functions
// =======================================================================================

// D0. Write data to flash
// 
void writeData2Flash (){
    file = SPIFFS.open("/master.csv", FILE_APPEND);
    if (!file) {
      Serial.println("There was an error opening the file for writing");
      lastFileWrite = "FAILED OPEN";
    } else {
      if (file.println(csvOutStr)){
        file.close();
        csvOutStr = ""; nSamples = 0;
        lastFileWrite = String(m.hour, DEC) + ":" + String(m.minute, DEC) + ":" + String(m.second, DEC);
      } else {
        lastFileWrite = "FAILED WRITE";
      }
    }
}


// D1. Processing LoRa Packets - this is the callback called when a packet is recevied
// The function:
//   Reads the entire packet
//   Checks if the packet starts with a "D"
//   Gets the drifter ID of the packet
//   Runs the packet decoder built into the Servant class object
//   Updates the distance and bearing of the Servant from the Master
//   Toggles the LED

void loraProcessRXData(int packetSize) {
  String packet = "";
  String packSize = String(packetSize, DEC);
  for (int i = 0; i < packetSize; i++) {
    packet += (char) LoRa.read();
  }
  String rssi = String(LoRa.packetRssi(), DEC);

  // Get ID and then send to class for decoding
  if (packet.substring(0, 1) == "D") {
    csvOutStr += packet; // Save all packets recevied (debugging purposes)
    int id = packet.substring(1, 3).toInt();
    Serial.print("GotID:" + String(id) + " ");
    s[id].ID = id;
    s[id].decode(packet);
    s[id].rssi = rssi.toInt();
    s[id].updateDistBear(m.lon, m.lat);

    Serial.println("RX from LoRa - decoding completed");
    Serial.println(String(s[id].ID));

    if (ledState == LOW) {
      digitalWrite(ledPin, HIGH);
      ledState = HIGH;
    } else {
      digitalWrite(ledPin, LOW);
      ledState = LOW;
    }
  }


}


// D2. Processing the onboard GPS
// The function:
//   Reads the GPS for 900ms  (allows interrupts)
//   Encodes the data via TinyGPS
//   Updates the Master data class object

void SerialGPSDecode(Stream &mySerial, TinyGPSPlus &myGPS) {
  // Read GPS and run decoder
  unsigned long start = millis();
  do
  {
    while (mySerial.available()) {
      myGPS.encode(mySerial.read());
    }
  } while (millis() - start < 500);

  if (gps.time.second() != gpsLastSecond) {
    // Update Master Data
    m.lon = gps.location.lng();
    m.lat = gps.location.lat();
    m.year = gps.date.year();
    m.month = gps.date.month();
    m.day = gps.date.day();
    m.hour = gps.time.hour();
    m.minute = gps.time.minute();
    m.second = gps.time.second();
    m.age = gps.location.age();
    gpsLastSecond = m.second;
    nSamples += 1;

    String tTime = String(m.hour) + ":" + String(m.minute) + ":" + String(m.second);
    masterData = "<tr><td>" + tTime + "</td><td>" + String(m.lon, 8) + "</td><td>" + String(m.lat, 8) + "</td><td>" + String(m.age) + "</td>";
    masterData += "<td><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"/getMaster\"> GET </a></td>";
    masterData += "<td>" + lastFileWrite + "</td>";
    masterData += "<td><a href=\"http://"+IpAddress2String(WiFi.softAPIP())+"/deleteMaster\"> ERASE </a></td>";
    masterData += "</tr>";
    
    // Update String to be written to file
    csvOutStr += String(m.hour) +  "," + String(m.minute) + "," + String(m.second) + "," + String(m.lon, 8) + "," + String(m.lat, 8) + "," + String(m.age) + "\n";

  }
}



// D3. Used to update sections of the webpages
//  Replaces placeholder with button section in your web page

String processor(const String& var) {
  if (var == "SERVANTS") {  return servantsData;  }
  if (var == "MASTER") {    return masterData;  }
  return String();
}


// D4. Reset NMEA serial output
//   Makes sure the onboard GPS has Serial NMEA output turned on

void resetGPSNMEAOutput(Stream &mySerial) {
  myGPS.begin(mySerial);
  myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}




// D5. String IP Address
String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") +\
  String(ipAddress[1]) + String(".") +\
  String(ipAddress[2]) + String(".") +\
  String(ipAddress[3])  ; 
}
