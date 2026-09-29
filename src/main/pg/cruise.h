/*
 * This file is part of Betaflight, licensed under the GNU General Public
 * License, version 3 or later. See <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include "pg/pg.h"

typedef struct cruiseConfig_s {
    uint8_t angleLimit;          // combined tilt, additionally bounded by the PID angle_limit
    uint8_t stickDeadband;       // PWM units, also used relative to the recovered channel snapshot
    uint8_t throttleThreshold;   // percent of the autopilot throttle interval
    uint8_t throttleHysteresis;  // percentage points
    uint16_t throttleDelayMs;
    uint8_t pitchCorrectionRate; // degrees/second
    uint8_t reserveBoost;        // percent of the captured normalized throttle
    uint16_t stickTimeMs;        // ANGLE-equivalent command / this interval
} cruiseConfig_t;

PG_DECLARE(cruiseConfig_t, cruiseConfig);
