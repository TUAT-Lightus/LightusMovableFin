#ifndef LIGHTUS_SENSORS_INA226_HPP
#define LIGHTUS_SENSORS_INA226_HPP
#include <Arduino.h>
#include <Wire.h>

#include "PowerMonitorSensorData.hpp"
namespace Lightus::Sensors {
class INA226 {
   public:
    void begin(TwoWire& wire);
    using DataType = PowerMonitorSensorData<float>;
    void setConfig(uint8_t ADDR_, uint16_t val);
    void setCalib(uint8_t ADDR_, uint16_t val);
    DataType busVoltage(uint8_t ADDR_);
    DataType read(uint8_t ADDR_);

   private:
    TwoWire* wire_;
    float SHUNT_RESISTA_      = 0.02;  // Ω
    float MV_PER_LSB_         = 1.25;
    uint8_t CONFIG_ADDR_      = 0x00;
    uint8_t SHUNTV_ADDR_      = 0x01;
    uint8_t BUS_VOLTAGE_ADDR_ = 0x02;
    uint8_t POWER_ADDR_       = 0x03;
    uint8_t CURRENT_ADDR_     = 0x04;
    uint8_t CALIB_ADDR_       = 0x05;
    uint8_t MASK_ADDR_        = 0x06;
    uint8_t ALERTL_ADDR_      = 0x07;
    float convertVoltageData_(uint16_t rawValue);
};
}  // namespace Lightus::Sensors
#if !defined(LIGHTUS_SENSORS_NO_INA226_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
extern Lightus::Sensors::INA226 ina226;
#endif
#endif
