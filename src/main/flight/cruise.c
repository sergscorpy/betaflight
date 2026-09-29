/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details. If not received, see <http://www.gnu.org/licenses/>.
 */

// CRUISE holds pilot-selected attitude targets; it has no navigation reference.
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "platform.h"

#ifdef USE_CRUISE
#include "build/debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "config/feature.h"
#include "config/config.h"
#include "drivers/time.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/alt_hold.h"
#include "flight/autopilot.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/position.h"
#include "pg/autopilot.h"
#include "pg/cruise.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "sensors/sensors.h"
#include "cruise.h"

PG_REGISTER_WITH_RESET_TEMPLATE(cruiseConfig_t, cruiseConfig, PG_CRUISE_CONFIG, 0);
PG_RESET_TEMPLATE(cruiseConfig_t, cruiseConfig,
    .angleLimit = 45, .stickDeadband = 5,
    .throttleThreshold = 50, .throttleHysteresis = 5,
    .throttleDelayMs = 3000, .pitchCorrectionRate = 5,
    .reserveBoost = 5, .stickTimeMs = 2000
);

static struct {
    bool active;
    bool reserve;
    bool pilotAvailable;
    bool rxLatched;
    bool fallbackAngle;
    bool switchWasOn;
    bool inhibitSwitch;
    bool modeSnapshotValid;
    bool overloadTimed;
    bool correcting;
    uint32_t modeSnapshot;
    float recoveredChannels[4];
    float angle[2];
    float angleLimit;
    float referenceThrottle;
    float reserveThrottle;
    timeUs_t controlAt;
    timeUs_t overloadAt;
    timeUs_t messageAt;
    const char *message;
} cruise;

static const boxId_e flightSwitches[] = {
    BOXANGLE, BOXHORIZON, BOXALTHOLD, BOXPOSHOLD, BOXAUTOPILOT,
    BOXGPSRESCUE, BOXPASSTHRU, BOXHEADFREE, BOXMAG, BOXCHIRP,
    BOXACROTRAINER, BOXLAUNCHCONTROL, BOXCRUISE
};

static uint32_t pilotModeMask(void)
{
    uint32_t mask = 0;
    for (unsigned i = 0; i < sizeof(flightSwitches) / sizeof(flightSwitches[0]); i++) {
        if (IS_RC_MODE_ACTIVE(flightSwitches[i])) {
            mask |= 1U << i;
        }
    }
    return mask;
}

static void notify(const char *message, timeUs_t now)
{
    cruise.message = message;
    cruise.messageAt = now;
}

static float limitAngle(void)
{
    return fminf(currentPidProfile->angle_limit, cruiseConfig()->angleLimit);
}

// With the other axis fixed, cos(tilt) = cos(roll) * cos(pitch).
static float axisLimit(float other, float limit)
{
    const float ratio = cosf(DEGREES_TO_RADIANS(limit)) / cosf(DEGREES_TO_RADIANS(other));
    return RADIANS_TO_DEGREES(acosf(constrainf(ratio, -1.0f, 1.0f)));
}

static void constrainTargets(void)
{
    const float limit = limitAngle();
    cruise.angle[FD_ROLL] = constrainf(cruise.angle[FD_ROLL], -limit, limit);
    const float pitchLimit = axisLimit(cruise.angle[FD_ROLL], limit);
    cruise.angle[FD_PITCH] = constrainf(cruise.angle[FD_PITCH], -pitchLimit, pitchLimit);
    cruise.angleLimit = limit;
}

static void captureAngles(void)
{
    cruise.angle[FD_ROLL] = attitude.values.roll * 0.1f;
    cruise.angle[FD_PITCH] = attitude.values.pitch * 0.1f;
    constrainTargets();
}

static bool supported(void)
{
    return ARMING_FLAG(ARMED) && wasThrottleRaised() && sensors(SENSOR_ACC)
        && !isFixedWing() && !featureIsEnabled(FEATURE_3D)
        && !isCrashFlipModeActive() && !isLaunchControlActive();
}

static bool altitudeAvailable(void)
{
    return autopilotAltitudeControlAvailable() && isAltitudeAvailable()
        && !IS_RC_MODE_ACTIVE(BOXCRUISERESERVE);
}

static void setReserve(bool reserve, timeUs_t now)
{
    if (cruise.reserve == reserve) {
        return;
    }
    cruise.reserve = reserve;
    cruise.overloadTimed = cruise.correcting = false;
    if (reserve) {
        cruise.reserveThrottle = constrainf(cruise.referenceThrottle * (1.0f + cruiseConfig()->reserveBoost * 0.01f), 0.0f, 1.0f);
        if (cruise.pilotAvailable) {
            for (unsigned i = 0; i < 4; i++) {
                cruise.recoveredChannels[i] = rcData[i];
            }
        }
        notify("RESERVE", now);
    } else {
        cruise.referenceThrottle = cruise.reserveThrottle;
        notify("ALT RESTORED", now);
    }
    // Even a brief loss must discard the old altitude target before re-entry.
    altHoldResetForCruise();
}

static bool engage(bool rxLoss, timeUs_t now)
{
    if (!supported() || FLIGHT_MODE(GPS_RESCUE_MODE | AUTOPILOT_MODE | PASSTHRU_MODE)) {
        notify("UNAVAILABLE", now);
        return false;
    }
    if (!cruise.active) {
        captureAngles();
        cruise.active = true;
        cruise.reserve = false;
        cruise.controlAt = now;
        cruise.overloadTimed = cruise.correcting = false;
        ENABLE_FLIGHT_MODE(CRUISE_MODE);
        notify(rxLoss ? "RX LOST" : "ACTIVE", now);
        setReserve(!altitudeAvailable(), now);
    }
    cruise.rxLatched |= rxLoss;
    if (rxLoss) {
        cruise.pilotAvailable = false;
        cruise.modeSnapshotValid = false;
        cruise.overloadTimed = cruise.correcting = false;
    }
    return true;
}

static void exitCruise(timeUs_t now)
{
    cruise.active = cruise.reserve = cruise.rxLatched = false;
    cruise.inhibitSwitch = IS_RC_MODE_ACTIVE(BOXCRUISE);
    cruise.overloadTimed = cruise.correcting = false;
    DISABLE_FLIGHT_MODE(CRUISE_MODE);
    // ANGLE/HORIZON/hold modes are decided again by core.c in this cycle.
    cruise.fallbackAngle = !(IS_RC_MODE_ACTIVE(BOXANGLE) || IS_RC_MODE_ACTIVE(BOXHORIZON)
        || IS_RC_MODE_ACTIVE(BOXALTHOLD) || IS_RC_MODE_ACTIVE(BOXPOSHOLD)
        || IS_RC_MODE_ACTIVE(BOXAUTOPILOT) || IS_RC_MODE_ACTIVE(BOXGPSRESCUE)
        || IS_RC_MODE_ACTIVE(BOXPASSTHRU));
    notify("EXIT", now);
}

void cruiseOnRxLoss(timeUs_t now)
{
    if (failsafeIsMonitoring() && ARMING_FLAG(ARMED)
        && (cruise.active || failsafeConfig()->failsafe_procedure == FAILSAFE_PROCEDURE_CRUISE)
        && !(IS_RC_MODE_ACTIVE(BOXFAILSAFE) && failsafeConfig()->failsafe_switch_mode == FAILSAFE_SWITCH_MODE_KILL)) {
        // Called before rxfail can reach the attitude controller, once per loss.
        if (!cruise.rxLatched) {
            engage(true, now);
        }
    }
}

void cruiseUpdateModes(timeUs_t now)
{
    if (!ARMING_FLAG(ARMED)) {
        const float reference = cruise.referenceThrottle;
        memset(&cruise, 0, sizeof(cruise));
        cruise.referenceThrottle = reference;
        DISABLE_FLIGHT_MODE(CRUISE_MODE);
        return;
    }

    const bool valid = rxAreFlightChannelsValid() && failsafeIsReceivingRxData()
        && !IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    const bool modeWasValid = cruise.modeSnapshotValid;
    const uint32_t modes = valid ? pilotModeMask() : cruise.modeSnapshot;
    const bool modeChanged = valid && modeWasValid && modes != cruise.modeSnapshot;
    const bool switchOn = IS_RC_MODE_ACTIVE(BOXCRUISE);
    if (valid && !switchOn) {
        cruise.inhibitSwitch = false;
    }
    if (modeChanged) {
        cruise.fallbackAngle = false;
    }

    if (valid && !cruise.pilotAvailable) {
        for (unsigned i = 0; i < 4; i++) {
            cruise.recoveredChannels[i] = rcData[i];
        }
        if (cruise.active && cruise.rxLatched) {
            notify("RX RESTORED", now);
        }
    }
    const bool justRecovered = valid && !cruise.pilotAvailable;
    cruise.pilotAvailable = valid;

    if (cruise.active) {
        if (!supported() || (failsafeIsActive() && failsafePhase() != FAILSAFE_CRUISE)) {
            exitCruise(now);
        } else {
            setReserve(!altitudeAvailable(), now);
            bool stickChanged = false;
            if (valid && !justRecovered && cruise.reserve) {
                for (unsigned i = 0; i < 4; i++) {
                    stickChanged |= fabsf(rcData[i] - cruise.recoveredChannels[i]) > cruiseConfig()->stickDeadband;
                }
            }
            if (modeChanged || stickChanged || (valid && !cruise.rxLatched && !switchOn)) {
                exitCruise(now);
            }
        }
    } else if (valid && switchOn && !cruise.switchWasOn && !cruise.inhibitSwitch) {
        engage(false, now);
        for (unsigned i = 0; i < 4; i++) {
            cruise.recoveredChannels[i] = rcData[i];
        }
    }

    if (valid) {
        cruise.switchWasOn = switchOn;
        cruise.modeSnapshot = modes;
        cruise.modeSnapshotValid = true;
    } else {
        cruise.modeSnapshotValid = false;
    }
}

void cruiseUpdateControl(timeUs_t now)
{
    if (!cruise.active) {
        return;
    }
    const float dt = constrainf(cmpTimeUs(now, cruise.controlAt) * 1e-6f, 0.0f, 0.05f);
    cruise.controlAt = now;
    const float limit = limitAngle();
    if (limit != cruise.angleLimit) {
        constrainTargets();
    }
    bool manualPitch = false;
    if (cruise.pilotAvailable && !cruise.reserve) {
        float delta[2] = {0};
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            const bool moved = fabsf(rcData[axis] - rxConfig()->midrc) > cruiseConfig()->stickDeadband;
            manualPitch |= axis == FD_PITCH && moved;
            if (moved && getMaxRcRate(axis) > 0.0f) {
                const float command = constrainf(getSetpointRate(axis) / getMaxRcRate(axis), -1.0f, 1.0f);
                delta[axis] = currentPidProfile->angle_limit * command * dt * 1000.0f / cruiseConfig()->stickTimeMs;
            }
        }
        // Axis-wise clipping keeps the other target intact and cannot wind up outside the envelope.
        for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
            if (delta[axis] == 0.0f) {
                continue;
            }
            const float max = axisLimit(cruise.angle[1 - axis], limit);
            cruise.angle[axis] = constrainf(cruise.angle[axis] + delta[axis], -max, max);
        }
    }

    if (manualPitch || cruise.reserve || !isAltHoldActive() || cruise.angle[FD_PITCH] == 0.0f) {
        cruise.overloadTimed = cruise.correcting = false;
    } else {
        const float pwm = MAX(rxConfig()->mincheck, PWM_RANGE_MIN)
            + getAutopilotThrottle() * (PWM_RANGE_MAX - MAX(rxConfig()->mincheck, PWM_RANGE_MIN));
        const float interval = autopilotConfig()->throttleMax - autopilotConfig()->throttleMin;
        const float utilization = interval > 0.0f ? 100.0f * (pwm - autopilotConfig()->throttleMin) / interval : 0.0f;
        const float start = cruiseConfig()->throttleThreshold;
        const float stop = fmaxf(0.0f, start - cruiseConfig()->throttleHysteresis);
        if (cruise.correcting && utilization <= stop) {
            cruise.correcting = cruise.overloadTimed = false;
        }
        if (!cruise.correcting) {
            if (utilization > start) {
                if (!cruise.overloadTimed) {
                    cruise.overloadAt = now;
                    cruise.overloadTimed = true;
                }
                if (cmpTimeUs(now, cruise.overloadAt) >= (timeDelta_t)cruiseConfig()->throttleDelayMs * 1000) {
                    cruise.correcting = true;
                    notify("PITCH REDUCED", now);
                }
            } else {
                cruise.overloadTimed = false;
            }
        }
        if (cruise.correcting) {
            const float pitch = cruise.angle[FD_PITCH];
            cruise.angle[FD_PITCH] = copysignf(fmaxf(0.0f, fabsf(pitch) - cruiseConfig()->pitchCorrectionRate * dt), pitch);
            if (cruise.angle[FD_PITCH] == 0.0f) {
                cruise.correcting = cruise.overloadTimed = false;
            }
        }
        DEBUG_SET(DEBUG_CRUISE, 4, lrintf(utilization * 10.0f));
    }
    DEBUG_SET(DEBUG_CRUISE, 0, lrintf(cruise.angle[FD_ROLL] * 10));
    DEBUG_SET(DEBUG_CRUISE, 1, lrintf(cruise.angle[FD_PITCH] * 10));
    DEBUG_SET(DEBUG_CRUISE, 2, attitude.values.roll);
    DEBUG_SET(DEBUG_CRUISE, 3, attitude.values.pitch);
    DEBUG_SET(DEBUG_CRUISE, 5, lrintf(cruise.referenceThrottle * 1000));
    DEBUG_SET(DEBUG_CRUISE, 6, cruise.reserve);
    DEBUG_SET(DEBUG_CRUISE, 7, cruise.correcting);
}

void cruiseObserveThrottle(float throttle)
{
    if (!cruise.active || (!cruise.reserve && isAltHoldActive())) {
        cruise.referenceThrottle = constrainf(throttle, 0.0f, 1.0f);
    }
}

bool cruiseIsActive(void)
{
    return cruise.active;
}

bool cruiseIsReserve(void)
{
    return cruise.active && cruise.reserve;
}

bool cruisePilotHasControl(void)
{
    return cruise.pilotAvailable && !cruise.reserve;
}

bool cruiseFallbackAngle(void)
{
    return cruise.fallbackAngle;
}

float cruiseGetAngle(int axis)
{
    return cruise.angle[axis];
}

float cruiseGetReserveThrottle(void)
{
    return cruise.reserveThrottle;
}

float cruiseGetReferenceThrottle(void)
{
    return cruise.reserve ? cruise.reserveThrottle : cruise.referenceThrottle;
}

const char *cruiseGetMessage(timeUs_t now)
{
    if (cruise.message && cmpTimeUs(now, cruise.messageAt) < 5000000) {
        return cruise.message;
    }
    return cruise.active ? (cruise.reserve ? "RESERVE" : (cruise.pilotAvailable ? "ACTIVE" : "RX LOST")) : NULL;
}
#endif
