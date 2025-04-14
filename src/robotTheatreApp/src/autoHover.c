#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "AUTO_HOVER"
#include "debug.h"

#include "log.h"
#include "param.h"
#include "supervisor.h"
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

1. check that all systems go: battery level is nominal and initial roll and pitch are flat surface.
2. Arm
3. Start minimum thrust e.g. 10%. Then slowly increase thrust up to 28%. 
4. Maintain this thrust level for a while (e.g. 5 seconds).
5. Decrease thrust slowly until thrust is below the minimum e.g. 10%
6. Wait then set thrust to zero.
7. Disarm.

NOTES:
- commanderSetSetpoint() must be sent within every COMMANDER_WDT_TIMEOUT_STABILIZE (500ms)
- for steps 3-8, each step needs a timeout. If timeout is reached, we need to go to the next step.
*/

#define COMMANDER_INTERVAL_MS 50 // 10ms to 50ms. must be less than COMMANDER_WDT_TIMEOUT_STABILIZE.

#define INCREMENT_THRUST_PERCENT 0.0005f; //0.001f // increment per interval

#define MIN_THRUST_PERCENT 0.2f // This thrust MUST be MORE than lift off thrust. To achieve minimal hover.
                                // need to hover (clear surface) in order for the stabilization to work
#define MAX_THRUST_PERCENT 0.28f //0.28f // 28% of full thrust. 
/*
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
*/

// we have no position info available. we can only control attitude and thrust.
// so all position modes are set to modeDisabled.
// percentThrust is 0.0f to 1.0f, where 1.0f is 100% thrust
static void setThrustSetpoint(setpoint_t *setpoint, float percentThrust)
{
  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;
  setpoint->mode.roll = modeAbs; //modeVelocity; 
  setpoint->attitude.roll = 0.0f;
  //setpoint->attitudeRate.roll = 0.0f; 
  setpoint->mode.pitch = modeAbs; //modeVelocity; 
  setpoint->attitude.pitch = 0.0f;
  //setpoint->attitudeRate.pitch = 0.0f;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = 0.0f; // we disable yaw control because we don't care
  
  if(percentThrust > MAX_THRUST_PERCENT) {
    percentThrust = MAX_THRUST_PERCENT;
  } else if(percentThrust < 0.0f) {
    percentThrust = 0.0f;    
  }
  setpoint->thrust = percentThrust* UINT16_MAX; 
}

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    vTaskDelay(M2T(3000)); // wait for power up stabilization
    offErrorLed(); // turn off error LED

    /*
    // Getting the Logging IDs of the state estimates
    logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");
    if (idHeightEstimate == -1) {
      DEBUG_PRINT("Failed to get log variable for altitude!\n");
      return;
    }
    logVarId_t idVelocityZ = logGetVarId("stateEstimate", "vz");
    if (idVelocityZ == -1) {
      DEBUG_PRINT("Failed to get log variable for velocity!\n");
      return;
    }

    surfaceAltitude = logGetFloat(idHeightEstimate); // get the altitude of the surface we are hovering over.
    // get roll and pitch. make sure they are close to 0.0f otherwise return.
    */
       
    if(!isFlyMode()) return;

    supervisorRequestArming(true); // request arming
    vTaskDelay(M2T(200)); 
    
    setpoint_t setpoint = {0};
    float curThrust = MIN_THRUST_PERCENT; // start with minimum thrust
    setThrustSetpoint(&setpoint, curThrust);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX); // set the thrust to the minimum thrust
    vTaskDelay(M2T(COMMANDER_INTERVAL_MS)); // wait for the next interval
    
    while(curThrust < MAX_THRUST_PERCENT) {
      // increase thrust
      curThrust += INCREMENT_THRUST_PERCENT; // increment thrust
      setThrustSetpoint(&setpoint, curThrust);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX); // set the thrust to the minimum thrust
      vTaskDelay(M2T(COMMANDER_INTERVAL_MS)); // wait for the next interval
    }
    
    for(int i = 0; i < 100; i++) { // stay here about 5 sec
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX); // set the thrust to the minimum thrust
      vTaskDelay(M2T(COMMANDER_INTERVAL_MS)); // wait for the next interval
    }
    
    while(curThrust > 0.0f) {
      // decrease thrust
      curThrust -= INCREMENT_THRUST_PERCENT; // increment thrust
      setThrustSetpoint(&setpoint, curThrust);
      commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX); // set the thrust to the minimum thrust
      vTaskDelay(M2T(COMMANDER_INTERVAL_MS)); // wait for the next interval
    }
    
    setThrustSetpoint(&setpoint, 0.0f);
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX); // set the thrust to the minimum thrust
    
    // flight landed
    while (1) {
        commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
        vTaskDelay(M2T(COMMANDER_INTERVAL_MS)); // Send setpoint every 100 ms
    }

}

  
