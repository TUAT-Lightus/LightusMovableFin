#ifndef LIGHTUS_SENSORS_NTCTHERMISTOR
#define LIGHTUS_SENSORS_NTCTHERMISTOR
#include <Arduino.h>

#include <cstdint>

#include "TempeartureData.hpp"
namespace Lightus::Sensors {
class NTCThermistor {
    uint8_t pin_;
    float B_;
    float R0_;
    float R1_;
    float Vin_;

   public:
    using DataType = Kelvin<float>;

    void begin(uint8_t pin, float B, float R0, float R1, float Vin = 3.3);

    void setPin(uint8_t pin);
    void setB(float B);
    void setR0(float R0);
    void setR1(float R1);
    void setVin(float Vin);

    DataType read();
};
}  // namespace Lightus::Sensors
#endif
