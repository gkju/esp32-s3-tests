#pragma once
#include <Arduino.h>

void handle_enc();
TrellisCallback blink(keyEvent evt);
uint32_t Wheel(byte WheelPos);
void HandleCommand(std::string_view &&cmd);
void handle_sliders();