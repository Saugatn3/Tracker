int called=0;
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
#include <SoftwareSerial.h> // Include the SoftwareSerial library

//const char apn[]      = "web"; 
//const char gprsUser[] = "";
//const char gprsPass[] = ""; 
#define WIFI_SSID "Seminar@lbef"
#define WIFI_PASSWORD "lbef##apu@@"
#define MAX_TIME_FOR_INTERNET_RESET 10
#define SerialAT Serial1 // Use the same hardware serial port for both GSM and GPS

SoftwareSerial SerialGPS(35, 34); // RX, TX pins for the SoftwareSerial (GPS)

int watchdogWiFiConnectCounter = 0;

// Create a TinyGPS++ object to parse GPS data
TinyGPSPlus gps;

TinyGsm modem(SerialAT);

// Set phone numbers, if you want to test SMS and Calls
#define CALL_TARGET "MOBILE_NUMBER"

void call() {
  Serial.println("Initializing modem...");
  modem.init();

  // Calling line identification presentation
  SerialAT.print("AT+CLIP=1\r\n");
  delay(10000);
   modem.init();
  Serial.print("Calling:");
  Serial.println(CALL_TARGET);

  bool res = modem.callNumber(CALL_TARGET);

  Serial.print("Call:");
  Serial.println(res? "ok" :"fail");

  // Hang up after 20 seconds
  delay(20000);

  res = modem.callHangup();
  Serial.println(res ? "Hang up: OK" : "Hang up: fail");
  called=1;
}

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  SerialGPS.begin(9600); // Initialize SoftwareSerial for GPS

  //if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    //Serial.println(" fail");
  //} else {
   // Serial.println("Pass");
    //}
    
  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);

    Serial.println("Connecting to WiFi...");
    // if SMS received, SMS = RESET, restart
    watchdogWiFiConnectCounter++;

    if (watchdogWiFiConnectCounter == MAX_TIME_FOR_INTERNET_RESET) {
      // TODO: Send SMS that GPRS connection failed.
      ESP.restart();
    }
  }
  Serial.println("Connected to WiFi!");

  if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  //Blynk.begin(auth, modem, apn, user, pass);
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
}

void loop() {
  if(!called){
   call();
    Serial.print("just called");
  }
  
  // Keep reading data from the GPS module
  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }

  if (gps.location.isValid()) {
    // gps.location.isUpdated()
    // Get latitude and longitude in degrees

    double latitude = round(gps.location.lat() * 10000.0) / 10000.0;
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
    Blynk.virtualWrite(V2, longitude, latitude);
    Blynk.virtualWrite(V1, speed);
  }
  //Blynk.begin(auth, modem, apn, user, pass);
  Blynk.run();  
}
