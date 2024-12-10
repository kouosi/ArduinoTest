#pragma once

namespace Buzzer {

void init();
void playPanic();
void playAccident();

} // namespace Buzzer

namespace Util {

__attribute__((noreturn)) void panic(const char* msg);
void ok();

} // namespace Util

namespace Gyro {

void init();
void checkTilt();
void fetch();

} // namespace Gyro

namespace Gps {

void init();
void fetch();
void print();

} // namespace Gps

namespace Sim {

void init();
void sendMessage();

} // namespace Sim


namespace Accident {
extern bool is_accident_detected;

void init();
void handler();
void check(float g_force_x, float g_force_y, float g_force_z, float gyro_x, float gyro_y, float gyro_z);

} // namespace Accident

