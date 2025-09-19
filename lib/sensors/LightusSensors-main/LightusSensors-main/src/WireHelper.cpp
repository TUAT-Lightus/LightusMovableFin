#include "WireHelper.hpp"

#include <Wire.h>
namespace Lightus::WireHelper {
void writeTo(TwoWire& wire, uint16_t address, uint16_t val) {
    wire.beginTransmission(address);
    wire.write(val);
    wire.endTransmission(false);
}

void setVal(TwoWire& wire, uint16_t add, uint16_t reg, uint8_t val) { setValRaw(wire, add, reg, &val, 1); }
void setVal16(TwoWire& wire, uint16_t add, uint16_t reg, uint16_t val, Endianness endian) {
    uint8_t buf[2];
    switch (endian) {
        case Endianness::BIG:
            buf[0] = (val >> 8) & 0xff;
            buf[1] = val & 0xff;
            break;
        case Endianness::LITTLE:
            buf[0] = val & 0xff;
            buf[1] = (val >> 8) & 0xff;
            break;
    }
    setValRaw(wire, add, reg, buf, sizeof(buf));
}
void setVal24(TwoWire& wire, uint16_t add, uint16_t reg, uint32_t val, Endianness endian) {
    uint8_t buf[3];
    switch (endian) {
        case Endianness::BIG:
            buf[0] = (val >> 16) & 0xff;
            buf[1] = (val >> 8) & 0xff;
            buf[2] = val & 0xff;
            break;
        case Endianness::LITTLE:
            buf[0] = val & 0xff;
            buf[1] = (val >> 8) & 0xff;
            buf[2] = (val >> 16) & 0xff;
            break;
    }
    setValRaw(wire, add, reg, buf, sizeof(buf));
}
void setValRaw(TwoWire& wire, uint16_t add, uint16_t reg, uint8_t* data, size_t len) {
    wire.beginTransmission(add);
    wire.write(reg);
    for (size_t i = 0; i < len; i++) {
        wire.write(data[i]);
    }
    wire.endTransmission(true);  // ここをfalseにするとesp32の仕様の問題を踏むので
}

uint8_t readData8(TwoWire& wire, uint16_t address1, uint16_t address2) {
    writeTo(wire, address1, address2);
    wire.requestFrom(address1, 1u);
    return wire.read();
}

uint16_t readData16(TwoWire& wire, uint16_t address1, uint16_t address2, Endianness endian) {
    uint8_t data[2];
    readDataRaw(wire, address1, address2, data, 2);
    return readDataFromRaw16(data, endian);
}

uint32_t readData24(TwoWire& wire, uint16_t add, uint16_t reg, Endianness endian) {
    uint8_t data[3];
    readDataRaw(wire, add, reg, data, 3);
    return readDataFromRaw24(data, endian);
}
size_t readDataRaw(TwoWire& wire, uint16_t address1, uint16_t address2, uint8_t* dst, size_t len, uint32_t timeout) {
    writeTo(wire, address1, address2);
    wire.requestFrom(address1, len);
    uint32_t startTime = millis();
    while (!wire.available()) {
        if (millis() - startTime >= timeout) {
            break;
        }
    }
    size_t i = 0;
    while (wire.available()) {
        dst[i] = wire.read();
        i++;
    }
    return i;
}
uint16_t readDataFromRaw16(uint8_t* rawData, Endianness endian) {
    uint16_t lsb, msb;
    switch (endian) {
        case Endianness::LITTLE:
            lsb = rawData[0];
            msb = rawData[1] << 8;
            break;
        case Endianness::BIG:
            msb = rawData[0] << 8;
            lsb = rawData[1];
            break;
    }
    uint16_t data16 = (msb | lsb);

    return data16;
}
uint32_t readDataFromRaw24(uint8_t* rawData, Endianness endian) {
    uint32_t msb, lsb, xlsb;
    switch (endian) {
        case Endianness::LITTLE:
            xlsb = rawData[0];
            lsb  = rawData[1] << 8;
            msb  = rawData[2] << 16;
            break;
        case Endianness::BIG:
            msb  = rawData[0] << 16;
            lsb  = rawData[1] << 8;
            xlsb = rawData[2];
            break;
    }

    uint32_t data32 = (msb | lsb | xlsb);

    return data32;
}
}  // namespace Lightus::WireHelper
