#include "INA226.hpp"

#include "Wire.h"
#include "WireHelper.hpp"

namespace Lightus::Sensors {
void INA226::begin(TwoWire& wire) { wire_ = &wire; }

void INA226::setConfig(uint8_t ADDR_, uint16_t val) {
    // Wire.beginTransmission(ADDR_);
    // Wire.write(CONFIG_ADDR_);
    // Wire.write(val >> 8);
    // Wire.write(val & 0xff);
    // Wire.endTransmission();
    WireHelper::setVal16(*wire_, ADDR_, CONFIG_ADDR_, val, WireHelper::Endianness::BIG);
}

void INA226::setCalib(uint8_t ADDR_, uint16_t val) {
    // Wire.beginTransmission(ADDR_);
    // Wire.write(CALIB_ADDR_);
    // Wire.write(val >> 8);
    // Wire.write(val & 0xff);
    // Wire.endTransmission();
    WireHelper::setVal16(*wire_, ADDR_, CALIB_ADDR_, val, WireHelper::Endianness::BIG);
}

INA226::DataType INA226::busVoltage(uint8_t ADDR_) {
    INA226::DataType result;
    result.voltage =
        convertVoltageData_(WireHelper::readData16(*wire_, ADDR_, BUS_VOLTAGE_ADDR_, WireHelper::Endianness::BIG));
    return result;
}
INA226::DataType INA226::read(uint8_t ADDR_) {
    INA226::DataType result;
    result.voltage = busVoltage(ADDR_).voltage;
    return result;
}

float INA226::convertVoltageData_(uint16_t rawValue) { return rawValue * MV_PER_LSB_ / 1000.0; }

}  // namespace Lightus::Sensors

#if !defined(LIGHTUS_SENSORS_NO_INA226_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
Lightus::Sensors::INA226 ina226;
#endif
