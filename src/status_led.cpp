#include "status_led.hpp"

// Define static members (pre-C++17 compatible)
bool StatusLed::_blueActive = false;
uint32_t StatusLed::_blueUntil = 0;
