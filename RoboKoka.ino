// ACCIDENT ALERT
// Dear User, an accident has been detected on your vehicle at [TIME].
// See the location on Google Maps: [LINK].
// Stay safe

#include <Adafruit_FONA.h>
#include <TinyGPS++.h>
#include <Arduino.h>
#include <SoftwareSerial.h>

namespace Gps {

// Inverse logic
#define GPS_RX_PIN 12
#define GPS_TX_PIN 11
#define GPS_BAUD_RATE 9600

TinyGPSPlus gps;
SoftwareSerial gps_serial(GPS_RX_PIN, GPS_TX_PIN);

void init() {
    Serial.println("Initializing GPS module...");
    gps_serial.begin(GPS_BAUD_RATE);
}

void fetch() {
    Serial.println("Fetching GPS location...");
    while (gps_serial.available()) {
        const char gps_serial_data = gps_serial.read();
        if (!gps.encode(gps_serial_data) && !gps.date.isUpdated() && !gps.location.isValid()) {
            continue;
        }
    }
}

void print() {
    fetch();
    if (gps.date.isUpdated()) {
        Serial.print("Latitude: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", Longitude: ");
        Serial.println(gps.location.lng(), 6);

        Serial.print("Date: ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());

        Serial.print("Time: ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.println(gps.time.second());

        Serial.print("Altitude: ");
        Serial.print(gps.altitude.meters());
        Serial.println(" meters");

        Serial.println("-----");
    }
}
} // namespace Gps

namespace Sim {

#define SIM800L_NUMBER "+9779745969290"
#define SIM800L_RXD_PIN 3
#define SIM800L_TXD_PIN 4
#define SIM800L_RESET_PIN 2
#define SIM800L_BAUD_RATE 9600

Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_RESET_PIN);
SoftwareSerial sim800l_serial = SoftwareSerial(SIM800L_TXD_PIN, SIM800L_RXD_PIN);


void printStrength() {
    int signal = sim800l.getRSSI();
    Serial.print("Signal strength: ");
    Serial.println(signal);
}
bool getReliableNetwork() {
    int i = 0;
    int rssiThreshold = -85;
    for (i = 0; i < 20; i++) {
        if (sim800l.getRSSI() > 20) {
            break;
        }

        printStrength();
        delay(1000);
    }

    return (i >= 15);
}

void init() {
    Serial.println("Initializing SIM800L Module...");
    sim800l_serial.begin(SIM800L_BAUD_RATE);
    if (!sim800l.begin(sim800l_serial)) {
        Serial.println(F("PANIC: Couldn\'t find SIM800L!"));
        while (true)
            ;
    }
    Serial.println(F("SIM800L initialized."));
    delay(500);
}

void sendMessage() {
    if (!getReliableNetwork()) {
        Serial.println(F("Low signal strength only, skipping SMS."));
        return;
    }
    char sendto[21] = "+9779745969290";
    char message[141] = "Hello world";

    Serial.println(F("Sending SMS..."));
    if (!sim800l.sendSMS(sendto, message)) {
        Serial.println(F("SMS failed to send."));
    } else {
        Serial.println(F("SMS sent successfully."));
    }
}


} // namespace Sim

/* Arduino Functions */
void setup() {
    Serial.begin(9600);
    // Gps::init();
    Sim::init();
    Sim::sendMessage();
}

void loop() {
    // Gps::print();
}
