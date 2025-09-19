#include "BME280.hpp"

#include <Wire.h>

#include <cmath>
#include <cstdint>
#include <serial_utils.hpp>

#include "WireHelper.hpp"
namespace Lightus::Sensors {
namespace {
namespace RegAddr {
constexpr uint8_t CTRL_HUM  = 0xf2;
constexpr uint8_t CTRL_MEAS = 0xf4;
constexpr uint8_t CONFIG    = 0xf5;
constexpr uint8_t PRESS_MSB = 0xf7;
constexpr uint8_t CALIB00   = 0x88;
constexpr uint8_t CALIB25   = 0xa1;
constexpr uint8_t CALIB26   = 0xe1;
}  // namespace RegAddr
}  // namespace

void BME280::begin(TwoWire& wire, TemperatureOversampling osrs_t, PressureOversampling osrs_p,
                   HumidityOversampling osrs_h, Mode mode, Tstandby t_sb, Filter filter, SPI3WireEnable spi3w_en) {
    wire_           = &wire;
    uint8_t osrs_t_ = static_cast<uint8_t>(osrs_t), osrs_p_ = static_cast<uint8_t>(osrs_p),
            osrs_h_ = static_cast<uint8_t>(osrs_h), mode_ = static_cast<uint8_t>(mode),
            t_sb_ = static_cast<uint8_t>(t_sb), filter_ = static_cast<uint8_t>(filter), spi3w_en_;
    uint8_t ctrl_meas_reg = (osrs_t_ << 5) | (osrs_p_ << 2) | mode_;  // 温度、圧力オーバーサンプリング
    uint8_t config_reg = (t_sb_ << 5) | (filter_ << 2) | spi3w_en_;  // スタンバイ時間とIIRフィルター設定
    uint8_t ctrl_hum_reg = osrs_h_;                                  // 湿度オーバーサンプリング

    WireHelper::setVal(*wire_, BME280_ADDRESS_, RegAddr::CTRL_HUM, ctrl_hum_reg);
    WireHelper::setVal(*wire_, BME280_ADDRESS_, RegAddr::CTRL_MEAS, ctrl_meas_reg);
    WireHelper::setVal(*wire_, BME280_ADDRESS_, RegAddr::CONFIG, config_reg);

    readTrim_();
}

void BME280::readTrim_() {
    byte inBuf[32];
    WireHelper::readDataRaw(*wire_, BME280_ADDRESS_, RegAddr::CALIB00, inBuf, 24);
    WireHelper::readDataRaw(*wire_, BME280_ADDRESS_, RegAddr::CALIB25, inBuf + 24, 1);
    WireHelper::readDataRaw(*wire_, BME280_ADDRESS_, RegAddr::CALIB26, inBuf + 24 + 1, 7);

    dig_T1_ = (inBuf[1] << 8) | inBuf[0];
    dig_T2_ = (inBuf[3] << 8) | inBuf[2];
    dig_T3_ = (inBuf[5] << 8) | inBuf[4];
    dig_P1_ = (inBuf[7] << 8) | inBuf[6];
    dig_P2_ = (inBuf[9] << 8) | inBuf[8];
    dig_P3_ = (inBuf[11] << 8) | inBuf[10];
    dig_P4_ = (inBuf[13] << 8) | inBuf[12];
    dig_P5_ = (inBuf[15] << 8) | inBuf[14];
    dig_P6_ = (inBuf[17] << 8) | inBuf[16];
    dig_P7_ = (inBuf[19] << 8) | inBuf[18];
    dig_P8_ = (inBuf[21] << 8) | inBuf[20];
    dig_P9_ = (inBuf[23] << 8) | inBuf[22];
    dig_H1_ = inBuf[24];
    dig_H2_ = (inBuf[26] << 8) | inBuf[25];
    dig_H3_ = inBuf[27];
    dig_H4_ = (inBuf[28] << 4) | (0x0F & inBuf[29]);
    dig_H5_ = (inBuf[30] << 4) | ((inBuf[29] >> 4) & 0x0F);
    dig_H6_ = inBuf[31];
}

float BME280::readPressure_(uint8_t* rawData) {
    uint32_t presRaw = (rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4);
    float result = computePres64_(presRaw) / 256.0;  // pa シフトしてしまうと小数点以下が失われるので、割り算する
    result *= 0.01;                                  // hpa
    return result;
}

float BME280::readTemperature_(uint8_t* rawData) {
    uint32_t TempRaw = (rawData[0] << 12) | (rawData[1] << 4) | (rawData[2] >> 4);
    return computeTemp64_(TempRaw) / 100.0;  // °C
}

float BME280::readHumidity_(uint8_t* rawData) {
    uint32_t HumRaw = (rawData[0] << 8 | rawData[1]);
    return computeHum64_(HumRaw) / 1024.0;
}

uint32_t BME280::computePres64_(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine_) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6_;
    var2 = var2 + ((var1 * (int64_t)dig_P5_) << 17);
    var2 = var2 + (((int64_t)dig_P4_) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3_) >> 8) + ((var1 * (int64_t)dig_P2_) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1_) >> 33;
    if (var1 == 0) {
        return 0;  // ゼロ除算による例外を避ける。
    }
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9_) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8_) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7_) << 4);
    return (uint32_t)p;
}

uint32_t BME280::computeTemp64_(int32_t adc_T) {
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1_ << 1))) * ((int32_t)dig_T2_)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1_)) * ((adc_T >> 4) - ((int32_t)dig_T1_))) >> 12) * ((int32_t)dig_T3_)) >>
           14;
    t_fine_ = var1 + var2;
    T       = (t_fine_ * 5 + 128) >> 8;
    return T;
}

uint32_t BME280::computeHum64_(int32_t adc_H) {
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine_ - ((int32_t)76800));
    v_x1_u32r =
        (((((adc_H << 14) - (((int32_t)dig_H4_) << 20) - (((int32_t)dig_H5_) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
         (((((((v_x1_u32r * ((int32_t)dig_H6_)) >> 10) *
              (((v_x1_u32r * ((int32_t)dig_H3_)) >> 11) + ((int32_t)32768))) >>
             10) +
            ((int32_t)2097152)) *
               ((int32_t)dig_H2_) +
           8192) >>
          14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1_)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

BME280::DataType BME280::read() {
    DataType result;
    uint8_t data[8];
    WireHelper::readDataRaw(*wire_, BME280_ADDRESS_, RegAddr::PRESS_MSB, data, 8);
    // 気圧データの補正には気温のデータ(t_fine_)が必要なので、先に気温のデータを補正する
    result.temperature = readTemperature_(data + 3);
    result.pressure    = readPressure_(data);
    result.humidity    = readHumidity_(data + 6);
    return result;
}
BME280::RawDataType BME280::readRaw() {
    RawDataType result;
    WireHelper::readDataRaw(*wire_, BME280_ADDRESS_, RegAddr::PRESS_MSB, result.data(), result.size());
    return result;
}
String BME280::readCompensationParametersJSON() {
    JsonDocument document;
    dumpCompensationParametersToJSON(document.as<JsonObject>());
    String result;
    serializeJson(document, result);
    return result;
}
#define assignToDict(dict, var) dict[#var] = var
void BME280::dumpCompensationParametersToJSON(JsonObject obj) {
    obj["version"] = 0;
    assignToDict(obj, dig_H6_);
    assignToDict(obj, dig_H1_);
    assignToDict(obj, dig_H3_);
    assignToDict(obj, dig_T2_);
    assignToDict(obj, dig_T3_);
    assignToDict(obj, dig_P2_);
    assignToDict(obj, dig_P3_);
    assignToDict(obj, dig_P4_);
    assignToDict(obj, dig_P5_);
    assignToDict(obj, dig_P6_);
    assignToDict(obj, dig_P7_);
    assignToDict(obj, dig_P8_);
    assignToDict(obj, dig_P9_);
    assignToDict(obj, dig_H2_);
    assignToDict(obj, dig_H4_);
    assignToDict(obj, dig_H5_);
    assignToDict(obj, dig_T1_);
    assignToDict(obj, dig_P1_);
}
void BME280::setAddress(uint8_t addr) { BME280_ADDRESS_ = addr; }
uint8_t BME280::address() const noexcept { return BME280_ADDRESS_; }
}  // namespace Lightus::Sensors

#if !defined(LIGHTUS_SENSORS_NO_BME280_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
Lightus::Sensors::BME280 bme280;
#endif