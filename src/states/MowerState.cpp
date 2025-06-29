#include "MowerState.h"
#include "../functions/Mower.h"
#include "../functions/MowerTypes.h"

void MowerState::handleError(Mower& mower, const char* errorMsg) {
#ifdef SERIAL_DEBUG
    Serial.print(F("Error in state "));
    Serial.print(getName());
    Serial.print(F(": "));
    Serial.println(errorMsg);
#endif
    // Qui potresti voler passare a uno stato di errore
    // mower.setState(mower.getErrorState());
}
