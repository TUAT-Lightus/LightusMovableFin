#include "MPU6050.hpp"

#include <cmath>
#include <serial_utils.hpp>

#include "WireHelper.hpp"
namespace Lightus::Sensors {
using WireHelper::Endianness;

// 無名名前空間: これで，別のcppファイル内の(無名名前空間の中の)RegAddrと衝突しない
namespace {
namespace RegAddr {
constexpr uint8_t CONFIG       = 0x1a;
constexpr uint8_t GYRO_CONFIG  = 0x1b;
constexpr uint8_t ACCEL_CONFIG = 0x1c;
constexpr uint8_t ACCEL_XOUT_H = 0x3b;
constexpr uint8_t GYRO_XOUT_H  = 0x43;
constexpr uint8_t PWR_MGMT_1   = 0x6b;
};  // namespace RegAddr
}  // namespace

void MPU6050::begin(TwoWire &wire, AccelRange accelRange, GyroRange gyroRange) {
    wire_       = &wire;
    accelRange_ = accelRange;
    gyroRange_  = gyroRange;
    WireHelper::setVal(*wire_, I2C_ADDR, RegAddr::GYRO_CONFIG, static_cast<uint8_t>(gyroRange_) << 3);
    WireHelper::setVal(*wire_, I2C_ADDR, RegAddr::ACCEL_CONFIG, static_cast<uint8_t>(accelRange_) << 3);
    WireHelper::setVal(*wire_, I2C_ADDR, RegAddr::CONFIG, 0x00);      // FSYNC: disabled, DLPF: disabled
    WireHelper::setVal(*wire_, I2C_ADDR, RegAddr::PWR_MGMT_1, 0x00);  // SLEEP: disabled <- デフォルトではenabled
}
MPU6050::DataType MPU6050::read() {
    MPU6050::DataType result;
    result.accel = readAccel();
    result.gyro  = readGyro();

    return result;
}
MPU6050::RawDataType MPU6050::read16() {
    MPU6050::RawDataType result;
    result.accel = readAccel16();
    result.gyro  = readGyro16();
    return result;
}
void MPU6050::dumpAccelCompensationParametersToJson_(JsonObject obj) { obj["G_PER_LSB"] = G_PER_LSB_(); }
void MPU6050::dumpGyroCompensationParametersToJson_(JsonObject obj) { obj["DPS_PER_LSB"] = DPS_PER_LSB_(); }
String MPU6050::readCompensationParametersJSON() {
    JsonDocument document;
    dumpCompensationParametersToJSON(document.as<JsonObject>());
    String result;
    serializeJson(document, result);
    return result;
}
void MPU6050::dumpCompensationParametersToJSON(JsonObject obj) {
    dumpAccelCompensationParametersToJson_(obj.createNestedObject("accel"));
    dumpGyroCompensationParametersToJson_(obj.createNestedObject("gyro"));
}
Vec3<int16_t> MPU6050::readAccel16() { return readAccel16_(); }
Vec3<int16_t> MPU6050::readGyro16() { return readGyro16_(); }
Vec3<float> MPU6050::convertAccelFrom16(Vec3<int16_t> accel16, AccelRange range) {
    return accel16 / static_cast<float>(LSB_PER_G_(range));
}
Vec3<float> MPU6050::convertGyroFrom16(Vec3<int16_t> gyro16, GyroRange range) { return gyro16 / LSB_PER_DPS_(range); }
Vec3<int16_t> MPU6050::readAccel16_() {
    int16_t ax16, ay16, az16;
    uint8_t buf[6];
    WireHelper::readDataRaw(*wire_, I2C_ADDR, RegAddr::ACCEL_XOUT_H, buf, 6);
    ax16 = WireHelper::readDataFromRaw16(buf, Endianness::BIG);
    ay16 = WireHelper::readDataFromRaw16(buf + 2, Endianness::BIG);
    az16 = WireHelper::readDataFromRaw16(buf + 4, Endianness::BIG);
    return Vec3<int16_t>(ax16, ay16, az16);
}
Vec3<int16_t> MPU6050::readGyro16_() {
    int16_t gx16, gy16, gz16;
    uint8_t buf[6];
    WireHelper::readDataRaw(*wire_, I2C_ADDR, RegAddr::GYRO_XOUT_H, buf, 6);
    gx16 = WireHelper::readDataFromRaw16(buf, Endianness::BIG);
    gy16 = WireHelper::readDataFromRaw16(buf + 2, Endianness::BIG);
    gz16 = WireHelper::readDataFromRaw16(buf + 4, Endianness::BIG);
    return Vec3<int16_t>(gx16, gy16, gz16);
}

int MPU6050::LSB_PER_G_() const { return LSB_PER_G_(accelRange_); }
int MPU6050::LSB_PER_G_(AccelRange accelRange) {
    // ±2g→16384LSB/g, ±4g→8192LSB/g, ±8g→4096LSB/g, ±16g→2048LSB/g
    switch (accelRange) {
        case AccelRange::PM2G:
            return 16384;
        case AccelRange::PM4G:
            return 8192;
        case AccelRange::PM8G:
            return 4096;
        case AccelRange::PM16G:
            return 2048;
        default:
            return 0;
    }
}

float MPU6050::G_PER_LSB_() const { return G_PER_LSB_(accelRange_); }
float MPU6050::G_PER_LSB_(AccelRange accelRange) { return 1.0 / LSB_PER_G_(accelRange); }

float MPU6050::LSB_PER_DPS_() const { return LSB_PER_DPS_(gyroRange_); }
float MPU6050::LSB_PER_DPS_(GyroRange gyroRange) {
    // ±2000°/s→16.4LSB/°/s, ±1000°/s→32.8LSB/°/s, ±500°/s→65.5LSB/°/s, ±250°/s→131LSB/°/s
    switch (gyroRange) {
        case GyroRange::PM2000DPS:
            return 16.4;
        case GyroRange::PM1000DPS:
            return 32.8;
        case GyroRange::PM500DPS:
            return 65.5;
        case GyroRange::PM250DPS:
            return 131;
        default:
            return std::nan("");
    }
}
float MPU6050::DPS_PER_LSB_() const { return DPS_PER_LSB_(gyroRange_); }
float MPU6050::DPS_PER_LSB_(GyroRange gyroRange) { return 1 / LSB_PER_DPS_(gyroRange); }

void MPU6050::setI2CAddress(uint8_t addr) { I2C_ADDR = addr; }
int MPU6050::absAccelMax() {
    switch (accelRange_) {
        case AccelRange::PM2G:
            return 2;
        case AccelRange::PM4G:
            return 4;
        case AccelRange::PM8G:
            return 8;
        case AccelRange::PM16G:
            return 16;
        default:
            return 0;
    }
}
int MPU6050::absGyroMax() {
    switch (gyroRange_) {
        case GyroRange::PM250DPS:
            return 250;
        case GyroRange::PM500DPS:
            return 500;
        case GyroRange::PM1000DPS:
            return 1000;
        case GyroRange::PM2000DPS:
            return 2000;
        default:
            return 0;
    }
}
MPU6050::AccelRange MPU6050::accelRangeFromAbsAccelMax(int absAccelMax) {
    switch (absAccelMax) {
        case 2:
            return AccelRange::PM2G;
        case 4:
            return AccelRange::PM4G;
        case 8:
            return AccelRange::PM8G;
        case 16:
            return AccelRange::PM16G;
        default:
            return AccelRange::INVALID;
    }
}
MPU6050::GyroRange MPU6050::gyroRangeFromAbsGyroMax(int absGyroMax) {
    switch (absGyroMax) {
        case 250:
            return GyroRange::PM250DPS;
        case 500:
            return GyroRange::PM500DPS;
        case 1000:
            return GyroRange::PM1000DPS;
        case 2000:
            return GyroRange::PM2000DPS;
        default:
            return GyroRange::INVALID;
    }
}
Vec3<float> MPU6050::readAccel() { return convertAccelFrom16(readAccel16_(), accelRange_); }
Vec3<float> MPU6050::readGyro() { return convertGyroFrom16(readGyro16_(), gyroRange_); }
uint8_t MPU6050::I2CAddress() const noexcept { return I2C_ADDR; }
}  // namespace Lightus::Sensors

#if !defined(LIGHTUS_SENSORS_NO_MPU6050_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
Lightus::Sensors::MPU6050 mpu6050;
#endif