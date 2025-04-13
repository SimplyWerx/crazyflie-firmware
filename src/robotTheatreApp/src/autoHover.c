#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "AUTO_HOVER"
#include "debug.h"

#include "log.h"
#include "param.h"

#include "commander.h"

#include "helloDeck.h"

void appMain() {
    DEBUG_PRINT("Waiting for activation ...\n");
    vTaskDelay(M2T(2000)); // wait for power up stabilization
    offErrorLed(); // turn off error LED

    // Getting the Logging IDs of the state estimates
    //logVarId_t idHeightEstimate = logGetVarId("stateEstimate", "z");

    while(1) {
      vTaskDelay(M2T(2000));
      DEBUG_PRINT("Hello World!\n");
      onErrorLed(); // turn on error LED
      vTaskDelay(M2T(2000)); 
      offErrorLed(); // turn off error LED
    }
}

  
