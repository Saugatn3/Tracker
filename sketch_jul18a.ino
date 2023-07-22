#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#define BLYNK_AUTH_TOKEN "h2K-pbD4qqrLRann094yiGSSeHfyD_GO"
#define WIFI_SSID "King's LANding"
#define WIFI_PASSWORD "lumbini@534"
#define MAX_TIME_FOR_INTERNET_RESET 10

int watchdogWiFiConnectCounter = 0;


// Create a TinyGPS++ object to parse GPS data
TinyGPSPlus gps;

TinyGsm modem(SerialAT);


void setup() {
  Serial.begin(115200); // Initialize Serial Monitor for debugging
  Serial2.begin(9600, SERIAL_8N1, 27, 26); // Initialize Serial2 for GPS communication

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
  }

  Blynk.run();
}
