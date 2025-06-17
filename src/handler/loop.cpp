#include "setup.h"
#include "loop.h"
#include "../../config.h"

void my_loop() {
    unsigned long currentTime = millis();

    // Update telemetry
    #ifdef ENABLE_WIFI
    //  wifi.update();
    //  wifi.sendTelemetry(currentState, getBatteryLevel(), rainSensor.isRainingNow(),
    //                     getSpeed(), getHeading());
    #endif



    // Update components
 
    // Update navigation if enabled
 
    // Update sensors

    #ifdef ENABLE_SCHEDULE
    //  scheduler.update();
    #endif


  // Macchina a stati principale
  switch (currentState) {
    case IDLE:
//      handleIdleState();
      break;

    case MOWING:
//      handleMowingState();
      break;

    case RETURNING_HOME:
//      handleReturningHomeState();
      break;

    case CHARGING:
//      handleChargingState();
      break;

    case ERROR:
//      handleErrorState();
      break;

    case MANUAL_CONTROL:
//      handleManualControlState();
      break;
  }

  // Piccolo delay per evitare sovraccarico CPU
  delay(MAIN_LOOP_DELAY);

}