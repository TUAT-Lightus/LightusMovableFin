#include "NTCThermistor.hpp"

#include <cmath>
namespace Lightus::Sensors {
void NTCThermistor::begin(uint8_t pin, float B, float R0, float R1, float Vin) {
    setPin(pin);
    setB(B);
    setR0(R0);
    setR1(R1);
    setVin(Vin);
}

void NTCThermistor::setPin(uint8_t pin) { this->pin_ = pin; }
void NTCThermistor::setB(float B) { this->B_ = B; }
void NTCThermistor::setR0(float R0) { this->R0_ = R0; }
void NTCThermistor::setR1(float R1) { this->R1_ = R1; }
void NTCThermistor::setVin(float Vin) { this->Vin_ = Vin; }

NTCThermistor::DataType NTCThermistor::read() {
    float Vout = analogReadMilliVolts(pin_) / 1000.0;
    float Rth  = (Vin_ / Vout - 1) * R1_;
    float invT = std::log(Rth / R0_) / B_ + 1 / 298.15;
    float T    = 1.0 / invT;
    return {T};
}
}  // namespace Lightus::Sensors
