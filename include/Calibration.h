#pragma once

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <HardwareSerial.h>
#include "MeanFilterLib.h"
#include "config.h"

/**
 * @class Calibration
 * @brief this class is in charge of locating the end efector of the mechanism in a referenced space.
 */
class Calibration{
    public:
        Calibration();
        void init();
        int start();
    private:
};