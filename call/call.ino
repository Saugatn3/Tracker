#define SIM800L_AXP192_VERSION_20200327

// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800          // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024   // Set RX buffer to 1Kb
#include "C:\Users\user\Desktop\sketch_jul18a\utilities.h"
#include <TinyGsmClient.h>
TinyGsm modem(SerialAT);

// Set phone numbers, if you want to test SMS and Calls
#define SMS_TARGET  "+9779865498607"
#define CALL_TARGET "+9779864063381"

void setup()
{
    // Start power management
    if (setupPMU() == false) {
        Serial.println("Setting power error");
    }

    // Some start operations
    setupModem();

    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);

    delay(6000);
}

void loop()
{
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    Serial.println("Initializing modem...");
    modem.init();

    // Swap the audio channels
    SerialAT.print("AT+CHFA=1\r\n");
    delay(2);

    //Set ringer sound level
    SerialAT.print("AT+CRSL=100\r\n");
    delay(2);

    //Set loud speaker volume level
    SerialAT.print("AT+CLVL=100\r\n");
    delay(2);

    // Calling line identification presentation
    SerialAT.print("AT+CLIP=1\r\n");
    delay(10000);

    Serial.print("Calling:");
    Serial.println(CALL_TARGET);

    bool res = modem.callNumber(CALL_TARGET);

    Serial.print("Call:");
    Serial.println(res ? "OK" : "fail");

    // Hang up after 20 seconds
    delay(20000);

    res = modem.callHangup();
    Serial.println(res ? "Hang up: OK" : "Hang up: fail");

    // Do nothing forevermore
    while (true) {
        modem.maintain();
    }
}
