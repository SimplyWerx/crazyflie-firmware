#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "AUTO_HOVER"
#include "debug.h"

#include "log.h"
#include "param.h"

#include "commander.h"

#include "helloDeck.h"

/*
1. check that all systems go: battery level is nominal and initial roll and pitch are flat surface.
2. Arm
3. Start minimum thrust e.g. 10%. Then slowly increase thrust up to 28%. Stop thrust increase when lift-off is detected.
4. Increase thrust slowly until tether max is reached i.e. height change (either height delta or z velocity) is fallen below a threshold. 
5. Maintain this thrust level for a while (e.g. 5 seconds).
6. Decrease thrust slowly until descent is detected.
7. Wait for a while (e.g. 5 seconds).
8. Decrease thrust slowly until thrust is below the minimum e.g. 10%
9. Wait then set thrust to zero.
10. Disarm.

NOTES:
- commanderSetSetpoint() must be sent within every COMMANDER_WDT_TIMEOUT_STABILIZE (500ms)
- for steps 3-8, each step needs a timeout. If timeout is reached, we need to go to the next step.
*/

#define COMMANDER_INTERVAL_MS 50 // 10ms to 50ms. must be less than COMMANDER_WDT_TIMEOUT_STABILIZE.

#define MIN_THRUST_PERCENT 0.1f // 10% of full thrust. This thrust MUST be less than lift off thrust

#define MAX_THRUST_PERCENT 0.28f // 28% of full thrust. 

static bool detectLiftOff() {
  logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");
  logVarId_t idVelocityZ = logGetVarId("stateEstimate", "vz");

  if (idHeightEstimate == -1 || idVelocityZ == -1) {
      DEBUG_PRINT("Failed to get log variables for altitude or velocity!\n");
      return false;
  }

  float altitude = logGetFloat(idHeightEstimate);
  float velocityZ = logGetFloat(idVelocityZ);

  if (altitude > 0.05f && velocityZ > 0.1f) { // Thresholds for lift-off
      DEBUG_PRINT("Lift-off detected! Altitude: %f m, Vertical velocity: %f m/s\n", altitude, velocityZ);
      return true;
  }
  return false;
}

// we have no position info available. we can only control attitude and thrust.
// so all position modes are set to modeDisabled.
// percentThrust is 0.0f to 1.0f, where 1.0f is 100% thrust
static void setVelocitySetpoint(setpoint_t *setpoint, float percentThrust)
{
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;
  setpoint->mode.yaw = modeDisable; // we disable yaw control because we don't care

  setpoint->attitude.roll = 0.0f;
  setpoint->attitude.pitch = 0.0f;
  
  if(percentThrust > MAX_THRUST_PERCENT) {
    percentThrust = MAX_THRUST_PERCENT;
  } else if(percentThrust < MIN_THRUST_PERCENT) {
    percentThrust = MIN_THRUST_PERCENT;    
  }
  setpoint->thrust = percentThrust * UINT16_MAX;

}

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    vTaskDelay(M2T(2000)); // wait for power up stabilization
    offErrorLed(); // turn off error LED

    // Getting the Logging IDs of the state estimates
    logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");
    if (idHeightEstimate == -1) {
      DEBUG_PRINT("Failed to get log variable for altitude!\n");
      return;
    }

    // get roll and pitch. make sure they are close to 0.0f otherwise return.

    while(1) {
      vTaskDelay(M2T(2000));
      float altitude = logGetFloat(idHeightEstimate);
      DEBUG_PRINT("Altitude: %f\n", altitude);
      onErrorLed(); // turn on error LED
      vTaskDelay(M2T(2000)); 
      offErrorLed(); // turn off error LED
    }
}

  
