/*
 * This file is part of Betaflight, licensed under the GNU General Public
 * License, version 3 or later. See <http://www.gnu.org/licenses/>.
 */

#include <cmath>
#include <cstring>
#include "gtest/gtest.h"
extern "C" {
#include "platform.h"
#include "build/debug.h"
#include "common/axis.h"
#include "config/feature.h"
#include "config/config.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/cruise.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "pg/autopilot.h"
#include "pg/cruise.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"
#include "rx/rx.h"
#include "sensors/sensors.h"

PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
uint8_t armingFlags;
uint16_t flightModeFlags;
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
attitudeEulerAngles_t attitude;
pidProfile_t profile;
pidProfile_t *currentPidProfile = &profile;
uint8_t debugMode;
int16_t debug[DEBUG16_VALUE_COUNT];
static bool boxes[CHECKBOX_ITEM_COUNT];
static bool valid, recovered, altitude, altActive, fsActive, accPresent;
static float throttle, rates[2];
static failsafePhase_e phase;
static timeUs_t now;
bool IS_RC_MODE_ACTIVE(boxId_e id) { return boxes[id]; }
bool rxAreFlightChannelsValid(void) { return valid; }
bool failsafeIsReceivingRxData(void) { return recovered; }
bool failsafeIsMonitoring(void) { return true; }
bool failsafeIsActive(void) { return fsActive; }
failsafePhase_e failsafePhase(void) { return phase; }
bool wasThrottleRaised(void) { return true; }
bool sensors(uint32_t mask) { return mask == SENSOR_ACC && accPresent; }
bool isFixedWing(void) { return false; }
bool isCrashFlipModeActive(void) { return false; }
bool isLaunchControlActive(void) { return false; }
bool featureIsEnabled(const uint32_t) { return false; }
bool autopilotAltitudeControlAvailable(void) { return true; }
bool isAltitudeAvailable(void) { return altitude; }
bool isAltHoldActive(void) { return altActive; }
void altHoldResetForCruise(void) { altActive = false; }
float getAutopilotThrottle(void) { return throttle; }
float getSetpointRate(int axis) { return rates[axis]; }
float getMaxRcRate(int) { return 670; }
uint16_t enableFlightMode(flightModeFlags_e mode) { return flightModeFlags |= mode; }
uint16_t disableFlightMode(flightModeFlags_e mode) { return flightModeFlags &= ~mode; }
timeUs_t micros(void) { return now; }
}

class CruiseTest : public ::testing::Test {
protected:
    void SetUp() override {
        now = 10000;
        armingFlags = 0;
        cruiseUpdateModes(now);
        memset(boxes, 0, sizeof(boxes));
        memset(&attitude, 0, sizeof(attitude));
        memset(rates, 0, sizeof(rates));
        for (unsigned i = 0; i < 4; i++) rcData[i] = 1500;
        valid = recovered = altitude = altActive = accPresent = true;
        fsActive = false;
        phase = FAILSAFE_IDLE;
        flightModeFlags = 0;
        armingFlags = ARMED;
        profile.angle_limit = 60;
        rxConfigMutable()->midrc = 1500;
        rxConfigMutable()->mincheck = 1000;
        autopilotConfigMutable()->throttleMin = 1100;
        autopilotConfigMutable()->throttleMax = 1900;
        failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_CRUISE;
        auto *cfg = cruiseConfigMutable();
        cfg->angleLimit = 45; cfg->stickDeadband = 5;
        cfg->throttleThreshold = 50; cfg->throttleHysteresis = 5;
        cfg->throttleDelayMs = 3000; cfg->pitchCorrectionRate = 5;
        cfg->reserveBoost = 5; cfg->stickTimeMs = 2000;
        throttle = 0.2f;
        cruiseObserveThrottle(throttle);
        cruiseUpdateModes(now);
    }
    void engage(float roll = 0, float pitch = 20) {
        attitude.values.roll = lroundf(roll * 10);
        attitude.values.pitch = lroundf(pitch * 10);
        boxes[BOXCRUISE] = true;
        cruiseUpdateModes(now);
        ASSERT_TRUE(cruiseIsActive());
    }
    void tick(int ms) {
        for (int i = 0; i < ms; i += 10) {
            now += 10000;
            cruiseUpdateControl(now);
        }
    }
};

TEST_F(CruiseTest, CaptureClipsCombinedTiltWithoutChangingRoll) {
    engage(30, 60);
    EXPECT_NEAR(cruiseGetAngle(FD_ROLL), 30, .001);
    EXPECT_NEAR(cruiseGetAngle(FD_PITCH), 35.2644, .001);
}
TEST_F(CruiseTest, ExcessNegativeRollClipsAndZerosPitch) {
    engage(-60, -30);
    EXPECT_NEAR(cruiseGetAngle(FD_ROLL), -45, .001);
    EXPECT_NEAR(cruiseGetAngle(FD_PITCH), 0, .01);
}
TEST_F(CruiseTest, RatesAccumulateLiveAndCenterRetainsTarget) {
    engage(0, 0);
    rcData[PITCH] = 1750; rates[PITCH] = 185;
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 30.0f * 185 / 670, .001);
    rcData[PITCH] = 1505;
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 30.0f * 185 / 670, .001);
}
TEST_F(CruiseTest, EnvelopeDoesNotWindUpAndReverseRespondsImmediately) {
    engage(30, 0);
    rcData[PITCH] = 2000; rates[PITCH] = 670;
    tick(5000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 35.2644, .001);
    rcData[PITCH] = 1000; rates[PITCH] = -670;
    tick(10);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 34.9644, .001);
    EXPECT_NEAR(cruiseGetAngle(ROLL), 30, .001);
}
TEST_F(CruiseTest, SustainedOverloadCorrectsAtFiveDegreesAndUsesHysteresis) {
    engage(10, 20); throttle = .51f;
    tick(3000);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), 20);
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 15, .001);
    throttle = .48f; // below 50%, above the 45% stop point (1460 PWM)
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 10, .001);
    throttle = .46f;
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 10, .001);
    EXPECT_FLOAT_EQ(cruiseGetAngle(ROLL), 10);
}
TEST_F(CruiseTest, BriefSpikeAndManualPitchResetDelay) {
    engage(); throttle = .51f;
    tick(2000); throttle = .4f; tick(1000); throttle = .51f; tick(2000);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), 20);
    rcData[PITCH] = 1506; rates[PITCH] = 0;
    tick(10);
    rcData[PITCH] = 1500;
    tick(3000);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), 20);
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), 15, .001);
}
TEST_F(CruiseTest, ReserveBoostIsAppliedOnceAndAltitudeRecovers) {
    altitude = false; engage();
    EXPECT_TRUE(cruiseIsReserve());
    EXPECT_NEAR(cruiseGetReserveThrottle(), .21f, .00001);
    cruiseObserveThrottle(.21f);
    cruiseUpdateModes(now);
    EXPECT_NEAR(cruiseGetReserveThrottle(), .21f, .00001);
    altitude = true;
    cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseIsReserve());
    EXPECT_NEAR(cruiseGetReferenceThrottle(), .21f, .00001);
}
TEST_F(CruiseTest, ForcedReservePreventsAutomaticReturn) {
    engage(); boxes[BOXCRUISERESERVE] = true;
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsReserve());
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsReserve());
    boxes[BOXCRUISERESERVE] = false;
    cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseIsReserve());
}
TEST_F(CruiseTest, LossCapturesAttitudeAndIgnoresSubstitutedCommands) {
    attitude.values.pitch = -200;
    valid = recovered = false;
    cruiseOnRxLoss(now);
    EXPECT_TRUE(cruiseIsActive());
    EXPECT_FALSE(cruisePilotHasControl());
    rcData[PITCH] = 2000; rates[PITCH] = 670;
    tick(1000);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), -20);
}
TEST_F(CruiseTest, NormalRecoveryDoesNotExitOnSticksButModeChangeDoes) {
    cruiseOnRxLoss(now);
    valid = recovered = false; cruiseUpdateModes(now);
    valid = recovered = true; cruiseUpdateModes(now);
    rcData[PITCH] += 50;
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsActive());
    boxes[BOXANGLE] = true;
    cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseIsActive());
}
TEST_F(CruiseTest, ReserveRecoveryNeedsNewMovementBeyondFivePwm) {
    altitude = false;
    valid = recovered = false; cruiseOnRxLoss(now); cruiseUpdateModes(now);
    valid = recovered = true; rcData[THROTTLE] = 1750;
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsActive());
    rcData[THROTTLE] += 5; cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsActive());
    rcData[THROTTLE] += 1; cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseIsActive());
    EXPECT_TRUE(cruiseFallbackAngle());
}
TEST_F(CruiseTest, ExitLatchesAngleUntilNextPilotModeChange) {
    engage(); boxes[BOXCRUISE] = false; cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseFallbackAngle());
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseFallbackAngle());
    boxes[BOXHORIZON] = true; cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseFallbackAngle());
}

TEST_F(CruiseTest, ReserveEntrySnapshotsCurrentSticksAndFreezesTargets) {
    engage();
    rcData[PITCH] = 1750; rates[PITCH] = 185;
    tick(500);
    const float pitch = cruiseGetAngle(PITCH);
    altitude = false;
    cruiseUpdateModes(now);
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsReserve());
    EXPECT_FALSE(cruisePilotHasControl());
    tick(1000);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), pitch);
    rcData[PITCH] += 6;
    cruiseUpdateModes(now);
    EXPECT_FALSE(cruiseIsActive());
}

TEST_F(CruiseTest, LossWhileAlreadyCruisingKeepsAccumulatedTargets) {
    engage(5, 25);
    attitude.values.roll = 200;
    attitude.values.pitch = 100;
    valid = recovered = false;
    cruiseOnRxLoss(now);
    EXPECT_FLOAT_EQ(cruiseGetAngle(ROLL), 5);
    EXPECT_FLOAT_EQ(cruiseGetAngle(PITCH), 25);
}

TEST_F(CruiseTest, NegativePitchProtectionMovesTowardLevelAndNeverRestoresAngle) {
    engage(12, -20);
    throttle = .51f;
    tick(4010);
    EXPECT_NEAR(cruiseGetAngle(PITCH), -14.95f, .001);
    throttle = .4f;
    tick(1000);
    EXPECT_NEAR(cruiseGetAngle(PITCH), -14.95f, .001);
    EXPECT_FLOAT_EQ(cruiseGetAngle(ROLL), 12);
}

TEST_F(CruiseTest, UtilitySwitchAndInvalidFlightModeValuesDoNotReleaseCruise) {
    engage();
    boxes[BOXBEEPERON] = true;
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsActive());
    valid = recovered = false;
    cruiseOnRxLoss(now);
    boxes[BOXANGLE] = true; // rxfail AUX replacement is not pilot intent.
    cruiseUpdateModes(now);
    EXPECT_TRUE(cruiseIsActive());
}
