#include "../../config.h"
#include "../../pin_config.h"

#ifdef ENABLE_PERIMETER
#include "PerimeterSensors.h"

PerimeterSensors::PerimeterSensors() :
    _signalStrength(0)
{
    // Constructor
}

PerimeterSensors::~PerimeterSensors() {
    // Destructor
}

void PerimeterSensors::begin() {
    // Entrambi i pin del filo perimetrale sono ingressi analogici
    pinMode(PERIMETER_SX_WIRE_PIN, INPUT);
    pinMode(PERIMETER_DX_WIRE_PIN, INPUT);
}

void PerimeterSensors::update() {
    int sx = analogRead(PERIMETER_SX_WIRE_PIN);
    int dx = analogRead(PERIMETER_DX_WIRE_PIN);
    _signalStrength = max(sx, dx);  // usa il segnale più forte
}

bool PerimeterSensors::isDetected() const {
    return _signalStrength > PERIMETER_SIGNAL_THRESHOLD;
}

int PerimeterSensors::getSignalStrength() const {
    return _signalStrength;
}

#endif // ENABLE_PERIMETER
