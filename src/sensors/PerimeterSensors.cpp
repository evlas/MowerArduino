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
    pinMode(PERIMETER_RX_PIN, INPUT);
    pinMode(PERIMETER_TX_PIN, OUTPUT);
    digitalWrite(PERIMETER_TX_PIN, LOW);
}

void PerimeterSensors::update() {
    _signalStrength = analogRead(PERIMETER_RX_PIN);
}

bool PerimeterSensors::isDetected() const {
    return _signalStrength > PERIMETER_SIGNAL_THRESHOLD;
}

int PerimeterSensors::getSignalStrength() const {
    return _signalStrength;
}

#endif // ENABLE_PERIMETER
