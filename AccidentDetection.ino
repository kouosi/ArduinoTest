#include <Arduino.h>
#include <HardwareSerial.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include "AccidentDetection.hpp"

/*****************************************************************************
 * Buzzer
 *****************************************************************************/
namespace Buzzer {

#define BUZZER_PIN 5

void init() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

void playPanic() {
    // Panic signal: rapid repeated pattern
    for (int i = 0; i < 3; i++) { // 15 short buzzes
        digitalWrite(BUZZER_PIN, HIGH);
        delay(100);
        digitalWrite(BUZZER_PIN, LOW);
        delay(100);
    }
}

void playAccident() {
    // Accident signal: slow repeated pattern
    for (int i = 0; i < 5; i++) { // 5 longer buzzes
        digitalWrite(BUZZER_PIN, HIGH);
        delay(500);
        digitalWrite(BUZZER_PIN, LOW);
        delay(300);
    }
}

} // namespace Buzzer

/*****************************************************************************
 * Util
 *****************************************************************************/
namespace Util {

__attribute__((noreturn)) void panic(const char* msg) {
    Buzzer::playPanic();
    while (true) {
    }
}

void ok() {
    Serial.println("....OK");
}

} // namespace Util

/*****************************************************************************
 * Gyro
 *****************************************************************************/
namespace Gyro {

#define ACCEL_THRESHOLD 4.0   // in g-force
#define GYRO_THRESHOLD 100.0  // in deg/sec
#define TILT_THRESHOLD 35.0   // Tilt angle in degrees
#define TILT_DURATION 2000    // 2 seconds
#define DETECTION_WINDOW 3000 // 3 seconds for confirming accident
#define USE_CALIBRATION 2

MPU6050 mpu;

float pitch = 0.0f, roll = 0.0f;
unsigned long last_check_time = 0;
unsigned long accident_confirmed_time = 0;
bool tilt_alert = false;
unsigned long tilt_start_time = 0;

float g_force_x = 0.0f, g_force_y = 0.0f, g_force_z = 0.0f, gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;

#if USE_CALIBRATION
// Complementary filter parameters
const float alpha = 0.98;

// Calibration offsets
float accel_offset_x = 0, accel_offset_y = 0, accel_offset_z = 0;
float gyro_offset_x = 0, gyro_offset_y = 0, gyro_offset_z = 0;

void calibrate() {
    Serial.println("Calibrating Gyro...");
    int num_samples = 1000;
    long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0;

    for (int i = 0; i < num_samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        delay(1);
    }

    accel_offset_x = ax_sum / (num_samples * 16384.0);
    accel_offset_y = ay_sum / (num_samples * 16384.0);
    accel_offset_z = az_sum / (num_samples * 16384.0);
    gyro_offset_x = gx_sum / (num_samples * 131.0);
    gyro_offset_y = gy_sum / (num_samples * 131.0);
    gyro_offset_z = gz_sum / (num_samples * 131.0);

    Util::ok();
}
#endif

void init() {
    Serial.print("Initializing MPU6050 module...");
    Wire.begin();
    mpu.initialize();
    if (mpu.testConnection()) {
        Util::panic("MPU6050 Initializing failed");
    }
    Util::ok();
#if USE_CALIBRATION
    calibrate();
#endif
}

void checkTilt() {
    // Check if tilt exceeds threshold
    if (abs(pitch) > TILT_THRESHOLD || abs(roll) > TILT_THRESHOLD) {
        if (!tilt_alert) {
            tilt_alert = true;
            tilt_start_time = millis();
            Serial.println("Excessive tilt detected! Monitoring duration...");
        } else if (millis() - tilt_start_time >= TILT_DURATION) {
            Accident::is_accident_detected = true;
            Serial.println("Tilt threshold exceeded for duration. Accident detected!");
            tilt_alert = false;
        }
    } else {
        tilt_alert = false;
    }
}

void fetch() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#if USE_CALIBRATION
    g_force_x = (ax / 16384.0) - accel_offset_x;
    g_force_y = (ay / 16384.0) - accel_offset_y;
    g_force_z = (az / 16384.0) - accel_offset_z;
    gyro_x = (gx / 131.0) - gyro_offset_x;
    gyro_y = (gy / 131.0) - gyro_offset_y;
    gyro_z = (gz / 131.0) - gyro_offset_z;
#else
    g_force_x = ax / 16384.0;
    g_force_y = ay / 16384.0;
    g_force_z = az / 16384.0;
    gyro_x = gx / 131.0;
    gyro_y = gy / 131.0;
    gyro_z = gz / 131.0;
#endif

#if USE_CALIBRATION
    // Low-pass filter for accelerometer
    static float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
    filtered_gx = (1 - alpha) * g_force_x + alpha * filtered_gx;
    filtered_gy = (1 - alpha) * g_force_y + alpha * filtered_gy;
    filtered_gz = (1 - alpha) * g_force_z + alpha * filtered_gz;

    // Complementary filter for pitch and roll
    float accel_pitch = atan2(filtered_gx, sqrt(filtered_gy * filtered_gy + filtered_gz * filtered_gz)) * 180.0 / PI;
    float accel_roll = atan2(filtered_gy, sqrt(filtered_gx * filtered_gx + filtered_gz * filtered_gz)) * 180.0 / PI;

    pitch = alpha * (pitch + gyro_x * 0.01) + (1 - alpha) * accel_pitch;
    roll = alpha * (roll + gyro_y * 0.01) + (1 - alpha) * accel_roll;
#else
    // Calculate tilt angles
    pitch = atan2(gForceX, sqrt(gForceY * gForceY + gForceZ * gForceZ)) * 180.0 / PI;
    roll = atan2(gForceY, sqrt(gForceX * gForceX + gForceZ * gForceZ)) * 180.0 / PI;
#endif

    // clang-format off
    Serial.print("Accel (g): "); Serial.print(g_force_x);
    Serial.print(", "); Serial.print(g_force_y);
    Serial.print(", "); Serial.print(g_force_z);
    Serial.print(" | Gyro (deg/sec): "); Serial.print(gyro_x);
    Serial.print(", "); Serial.print(gyro_y);
    Serial.print(", "); Serial.print(gyro_z);
    Serial.print(" | Tilt (pitch, roll): "); Serial.print(pitch);
    Serial.print(", "); Serial.println(roll);
    // clang-format on

    checkTilt();
    Accident::check(g_force_x, g_force_y, g_force_z, gyro_x, gyro_y, gyro_z);
}

} // namespace Gyro

/*****************************************************************************
 * GPS
 *****************************************************************************/
namespace Gps {

// Inverse logic
#define GPS_RX_PIN 12
#define GPS_TX_PIN 11

TinyGPSPlus gps;
SoftwareSerial gps_serial(GPS_RX_PIN, GPS_TX_PIN);

void init() {
    Serial.println("Initializing GPS module...");
    gps_serial.begin(9600);
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
        // clang-format off
        Serial.print("Latitude: "); Serial.print(gps.location.lat(), 6);
        Serial.print(", Longitude: "); Serial.println(gps.location.lng(), 6);

        Serial.print("Date: "); Serial.print(gps.date.day());
        Serial.print("/"); Serial.print(gps.date.month());
        Serial.print("/"); Serial.println(gps.date.year());

        Serial.print("Time: "); Serial.print(gps.time.hour());
        Serial.print(":"); Serial.print(gps.time.minute());
        Serial.print(":"); Serial.println(gps.time.second());

        Serial.print("Altitude: "); Serial.print(gps.altitude.meters()); Serial.println(" meters");
        Serial.println("-----");
        // clang-format on
    }
}
} // namespace Gps

namespace Sim {

#define SIM900A_NUMBER "+9779745969290"
#define SIM900A_RXD_PIN 3
#define SIM900A_TXD_PIN 4
#define SIM900A_RESET_PIN 2
#define SIM900A_TIMEOUT 5000

SoftwareSerial sim900a(2, 3);

bool checkResponse(const String& expected) {
    delay(1000);
    unsigned long timeout = millis() + SIM900A_TIMEOUT;
    String response = "";

    while (millis() < timeout) {
        while (sim900a.available()) {
            char c = sim900a.read();
            response += c;
            if (response.indexOf(expected) != -1) {
                return true;
            }
        }
    }
    return false;
}

void init() {
    Serial.print("Initializing sim900a Module...");
    sim900a.begin(9600);

    sim900a.println("AT");
    if (checkResponse("OK")) {
        Util::ok();
    } else {
        Util::panic("Error: Failed to connect to SIM module");
    }

    sim900a.println("AT+CMGF=1");
    if (checkResponse("OK")) {
        Serial.println("SMS mode set successfully.");
    } else {
        Util::panic("Error: Failed to set SIM to SMS mode.");
    }
}

void sendMessage() {
    Serial.println("Sending SMS alert...");

    sim900a.println("AT+CMGS=\"" SIM900A_NUMBER "\"");
    if (checkResponse(">")) {
        Serial.println("Recipient phone number set successfully.");
    } else {
        Util::panic("Error: Failed to set recipient's phone number.");
    }

    String message = "ACCIDENT ALERT!\n"
                     "Dear User, an accident has been detected on your vehicle at [TIME]."
                     "See the location on Google Maps: [LINK]. \n"
                     "Stay safe!";

    sim900a.println(message);
    sim900a.write(26); // (C-z)
    if (checkResponse("OK")) {
        Serial.println("SMS alert sent successfully.");
    } else {
        Serial.println("Error: Failed to send SMS alert.");
    }

    Util::ok();
}

} // namespace Sim

/*****************************************************************************
 * Accident
 *****************************************************************************/
namespace Accident {

#define ACCIDENT_RESET_PIN 7
#define CAR_RESET_DURATION 8000 // 8 sec

bool is_accident_detected = false;

void init() {
    pinMode(ACCIDENT_RESET_PIN, INPUT);
}

void handler() {
    Serial.println("\n** Accident detected **\n");
    Serial.println("Activating Buzzer");

    unsigned long press_start = 0;
    while (true) {
        Buzzer::playAccident();
        if (digitalRead(ACCIDENT_RESET_PIN) == HIGH) {
            if (!press_start) {
                press_start = millis();
            }

            if ((millis() - press_start) > CAR_RESET_DURATION) {
                // TODO: Reset whole car
                return;
            }
        } else {
            if (press_start > 0 && (millis() - press_start) < CAR_RESET_DURATION) {
                Sim::sendMessage();
                is_accident_detected = false;
                return;
            }
            press_start = 0;
        }
    }
}

void check(float g_force_x, float g_force_y, float g_force_z, float gyro_x, float gyro_y, float gyro_z) {
    // Check if accelerometer or gyroscope values exceed thresholds
    bool accel_exceeded =
        (abs(g_force_x) > ACCEL_THRESHOLD || abs(g_force_y) > ACCEL_THRESHOLD || abs(g_force_z) > ACCEL_THRESHOLD);
    bool gyro_exceeded = (abs(gyro_x) > GYRO_THRESHOLD || abs(gyro_y) > GYRO_THRESHOLD || abs(gyro_z) > GYRO_THRESHOLD);

    if (accel_exceeded || gyro_exceeded) {
        if (!is_accident_detected) {
            is_accident_detected = true;
            Gyro::accident_confirmed_time = millis();
            Serial.println("Potential accident detected!");
        }
    }
}

} // namespace Accident

/*****************************************************************************
 *Arduino
 *****************************************************************************/
void setup() {
    Serial.begin(9600);
    Sim::init();
    Gps::init();
    Buzzer::init();
    Gyro::init();
    Sim::sendMessage();
}

void loop() {
    if (millis() - Gyro::last_check_time >= DETECTION_WINDOW) {
        Gyro::last_check_time = millis();
        Gyro::fetch();
    }
    if (Accident::is_accident_detected) {
        Accident::handler();
    }
}
