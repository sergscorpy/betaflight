/*
 * This file is part of Betaflight, licensed under the GNU General Public
 * License, version 3 or later. See <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include "common/time.h"

#ifdef USE_CRUISE
void cruiseOnRxLoss(timeUs_t now);
void cruiseUpdateModes(timeUs_t now);
void cruiseUpdateControl(timeUs_t now);
void cruiseObserveThrottle(float throttle);
bool cruiseIsActive(void);
bool cruiseIsReserve(void);
bool cruisePilotHasControl(void);
bool cruiseFallbackAngle(void);
float cruiseGetAngle(int axis);
float cruiseGetReserveThrottle(void);
float cruiseGetReferenceThrottle(void);
const char *cruiseGetMessage(timeUs_t now);
#else
static inline bool cruiseIsActive(void) { return false; }
static inline bool cruiseIsReserve(void) { return false; }
static inline bool cruisePilotHasControl(void) { return true; }
static inline bool cruiseFallbackAngle(void) { return false; }
#endif
