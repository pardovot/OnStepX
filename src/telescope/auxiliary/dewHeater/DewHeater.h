// -----------------------------------------------------------------------------------
// Dew Heater control
#pragma once

#include "../../../common.h"

#ifndef DEW_HEATER_PULSE_WIDTH_MS
  #define DEW_HEATER_PULSE_WIDTH_MS 2000
#endif

class DewHeater {
  public:
    void init(int index, bool validKey);

    void poll(float deltaAboveDewPointC);

    float getZero();
    void setZero(float t);

    float getSpan();
    void setSpan(float t);

    bool isEnabled();
    void enable(bool state);

    bool isOn();

  private:
    unsigned long lastHeaterCycle = 0;
    unsigned long currentTime = 0;

    bool heaterOn = false;
    bool enabled = false;

    float zero = -5;
    float span = 15;

    int index = 0;
};
