#include <SoftwareSerial.h>

#define SIM800L_AXP192_VERSION_20200327
#define BLYNK_TEMPLATE_ID "TMPL6HSATnYDn"
#define BLYNK_TEMPLATE_NAME "gps"
#define BLYNK_AUTH_TOKEN "UZzKU4HnX8oI8U8VDSIiRx8GJsMayEzH"
#define TINY_GSM_MODEM_SIM800
#include "C:\Users\user\Desktop\sketch_jul18a\utilities.h"
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

SoftwareSerial gsmSerial(MODEM_RX, MODEM_TX);

#define TINY_GSM_RX_BUFFER 1024
#define WIFI_SSID "King's LANding"
#define WIFI_PASSWORD "lumbini@534"
#define MAX_TIME_FOR_INTERNET_RESET 10
#define SerialAT Serial2
int watchdogWiFiConnectCounter = 0;

#define CALL_TARGET "+9779864063381"
// Create a TinyGPS++ object to parse GPS data
TinyGPSPlus gps;

//TinyGsm modem(Serial1);
TinyGsm modem(SerialAT);

void call(){
    gsmSerial.println("Initializing modem...");
    modem.init();

    // Swap the audio channels
    gsmSerial.print("AT+CHFA=1\r\n");
    delay(2);
    //Set ringer sound level
    gsmSerial.print("AT+CRSL=100\r\n");
    delay(2);

    //Set loud speaker volume level
    gsmSerial.print("AT+CLVL=100\r\n");
    delay(2);

    // Calling line identification presentation
    gsmSerial.print("AT+CLIP=1\r\n");
    delay(10000);

    gsmSerial.print("Calling:");
    gsmSerial.println(CALL_TARGET);

    bool res = modem.callNumber(CALL_TARGET);

    gsmSerial.print("Call:");
    gsmSerial.println(res ? "OK" : "fail");

    // Hang up after 20 seconds
    delay(20000);
    res=modem.callHangup();
    gsmSerial.print("Call:");
    gsmSerial.println(res ? "OK" : "fail");
    ESP.restart();
}


void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  SerialAT.begin(9600, SERIAL_8N1, 27, 26); // Initialize Serial2 for GPS communication
  gsmSerial.begin(9600); 

  if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    // Some start operations
    setupModem();

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);

    Serial.println("Connecting to WiFi...");
    // if sms received , sms = RESET , restart
    watchdogWiFiConnectCounter++;

    if (watchdogWiFiConnectCounter == MAX_TIME_FOR_INTERNET_RESET) {
      // TODO : Send sms that gprs connection failed.
      ESP.restart();
     }

  }
  Serial.println("Connected to WiFi!");

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  // Keep reading data from the GPS module
  call();
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }

  // Check if we have valid GPS data
  if (gps.location.isValid()) {

    // gps.location.isUpdated()
    // Get latitude and longitude in degrees

    double latitude = round(gps.location.lat()* 10000.0) / 10000.0;
    double longitude = round(gps.location.lng() * 10000.0) / 10000.0;

    // Get altitude in meters above sea level
    double altitude = gps.altitude.meters();
    double speed = gps.speed.kmph();

    // Print GPS data to Serial Monitor (for debugging)
    Serial.print("Latitude: ");
    Serial.print(latitude); // Print latitude with 6 decimal places
    Serial.print(", Longitude: ");
    Serial.print(longitude); // Print longitude with 6 decimal places
    Serial.print(", Altitude: ");
    Serial.print(altitude);
    Serial.print(" meters");
      Serial.print(", Speed: ");
    Serial.print(speed);
    Serial.println(" kmph");
    // Send latitude and longitude to Blynk virtual pins V1 and V2
    Blynk.virtualWrite(V3, altitude);
    Blynk.virtualWrite(V4, speed);
    Blynk.virtualWrite(V2,longitude,latitude);
        Blynk.virtualWrite(V1,speed);

  }

  Blynk.run();
}
