//#include "BNO055.hpp"
//
//#include <cmath>
//#include <serial_utils.hpp>
//
//#include "WireHelper.hpp"
//namespace Lightus::Sensors {
//// 無名名前空間: これで，別のcppファイル内の(無名名前空間の中の)RegAddrと衝突しない
//namespace {
//namespace RegAddr {
//namespace Page0 {
//constexpr uint8_t ACCD_X_LSB = 0x08;
//constexpr uint8_t MAGD_X_LSB = 0x0E;
//constexpr uint8_t GYRD_X_LSB = 0x14;
//constexpr uint8_t PAGE_ID    = 0x07;
//constexpr uint8_t OPR_MODE   = 0x3D;
//}  // namespace Accel
//namespace Page1 {
//constexpr uint8_t RATE_X_LSB = 0x02;
//constexpr uint8_t RANGE      = 0x0f;
//constexpr uint8_t RATE_HBW   = 0x13;
//}  // namespace Gyro
//
//};  // namespace RegAddr
//}  // namespace
//using WireHelper::Endianness;
//void BNO055::begin(TwoWire &wire, AccelRange accelRange, GyroRange gyroRange, MagneticRate magneticRate,
//                   FilterConfig filter) {
//    wire_         = &wire;
//    accelRange_   = accelRange;
//    gyroRange_    = gyroRange;
//    magneticRate_ = magneticRate;
//    WireHelper::setVal(*wire_, BNO_ADDRESS_, RegAddr::Page0::OPR_MODE, static_cast<uint8_t>(0b0111));
//    // WireHelper::setVal(*wire_, ACCL_ADDRESS_, RegAddr::Accel::ACCD_HBW, static_cast<uint8_t>(filter) << 7);
//    // WireHelper::setVal(*wire_, GYRO_ADDRESS_, RegAddr::Gyro::RANGE, static_cast<uint8_t>(gyroRange_));
//    // WireHelper::setVal(*wire_, GYRO_ADDRESS_, RegAddr::Gyro::RATE_HBW, static_cast<uint8_t>(filter) << 7);
//    // WireHelper::setVal(*wire_, MAGN_ADDRESS_, 0x4B, 0b1000'0011);  // softe reset
//    delay(100);  // リセット完了を待つ．これがないと電源を落として再起動したときに地磁気が読めなくなる
//    // WireHelper::setVal(*wire_, MAGN_ADDRESS_, 0x4C, static_cast<uint8_t>(magneticRate_) << 3);
//    // WireHelper::setVal(*wire_, MAGN_ADDRESS_, 0x4E, 0b1000'0100);
//    // WireHelper::setVal(*wire_, MAGN_ADDRESS_, 0x51, 0b0000'0100);
//    // WireHelper::setVal(*wire_, MAGN_ADDRESS_, 0x52, 0b0000'1110);
//
//    // ReadTrimMag_();
//}
//BNO055::DataType BNO055::read() {
//    BNO055::DataType result;
//    result.accel = readAccel();
//    result.gyro  = readGyro();
//    result.mag   = readMag();
//
//    return result;
//}
//BNO055::RawDataType BNO055::read16() {
//    BNO055::RawDataType result;
//    result.accel = readAccel16();
//    result.gyro  = readGyro16();
//    result.mag   = readMag16();
//    return result;
//}
//#define assignToDict(dict, var) dict[#var] = var
//void BNO055::BMM150TrimData::toJSON(JsonObject obj) {
//    assignToDict(obj, dig_x1);
//    assignToDict(obj, dig_y1);
//    assignToDict(obj, dig_x2);
//    assignToDict(obj, dig_y2);
//    assignToDict(obj, dig_z1);
//    assignToDict(obj, dig_z2);
//    assignToDict(obj, dig_z3);
//    assignToDict(obj, dig_z4);
//    assignToDict(obj, dig_xy1);
//    assignToDict(obj, dig_xy2);
//    assignToDict(obj, dig_xyz1);
//}
//void BNO055::dumpAccelCompensationParametersToJson_(JsonObject obj) { obj["G_PER_LSB"] = G_PER_LSB_(); }
//void BNO055::dumpGyroCompensationParametersToJson_(JsonObject obj) { obj["DPS_PER_LSB"] = DPS_PER_LSB_(); }
//void BNO055::dumpMagCompensationParametersToJson_(JsonObject obj) { magTrimData_.toJSON(obj); }
//String BNO055::readCompensationParametersJSON() {
//    JsonDocument document;
//    dumpCompensationParametersToJSON(document.as<JsonObject>());
//    String result;
//    serializeJson(document, result);
//    return result;
//}
//void BNO055::dumpCompensationParametersToJSON(JsonObject obj) {
//    obj["version"] = 0;
//    dumpAccelCompensationParametersToJson_(obj.createNestedObject("accel"));
//    dumpGyroCompensationParametersToJson_(obj.createNestedObject("gyro"));
//    dumpMagCompensationParametersToJson_(obj.createNestedObject("mag"));
//}
//Vec3<int16_t> BNO055::readAccel16() { return readAccel16_(); }
//Vec3<int16_t> BNO055::readGyro16() { return readGyro16_(); }
//Vec3<int16_t> BNO055::readMag16() { return readMag16_(); }
//Vec3<float> BNO055::convertAccelFrom16(Vec3<int16_t> accel16, AccelRange range) {
//    return (accel16)*G_PER_LSB_(range) / 1000;
//}
//Vec3<float> BNO055::convertGyroFrom16(Vec3<int16_t> gyro16, GyroRange range) { return gyro16 * DPS_PER_LSB_(range); }
//Vec3<int16_t> BNO055::readAccel16_() {
//    int16_t ax16, ay16, az16;
//    uint8_t buf[6];
//    WireHelper::readDataRaw(*wire_, BNO_ADRESS_, RegAddr::Page0::ACCD_X_LSB, buf, 6);
//    ax16 = WireHelper::readDataFromRaw16(buf, Endianness::LITTLE);
//    ay16 = WireHelper::readDataFromRaw16(buf + 2, Endianness::LITTLE);
//    az16 = WireHelper::readDataFromRaw16(buf + 4, Endianness::LITTLE);
//    return Vec3<int16_t>(ax16, ay16, az16) >> 4;
//}
//Vec3<int16_t> BNO055::readGyro16_() {
//    int16_t gx16, gy16, gz16;
//    uint8_t buf[6];
//    WireHelper::readDataRaw(*wire_, BNO_ADDRESS_, RegAddr::Page0::GYRD_X_LSB, buf, 6);
//    gx16 = WireHelper::readDataFromRaw16(buf, Endianness::LITTLE);
//    gy16 = WireHelper::readDataFromRaw16(buf + 2, Endianness::LITTLE);
//    gz16 = WireHelper::readDataFromRaw16(buf + 4, Endianness::LITTLE);
//    return Vec3<int16_t>(gx16, gy16, gz16);
//}
//Vec3<int16_t> BNO055::readMag16_() {
//    int16_t mx16, my16, mz16;
//    uint8_t buf[6];
//    WireHelper::readDataRaw(*wire_, BNO_ADDRESS_, RegAddr::Page0::MAGD_X_LSB, buf, 6);
//    mx16 = WireHelper::readDataFromRaw16(buf, Endianness::LITTLE);
//    my16 = WireHelper::readDataFromRaw16(buf + 2, Endianness::LITTLE);
//    mz16 = WireHelper::readDataFromRaw16(buf + 4, Endianness::LITTLE);
//    return Vec3<int16_t>(mx16, my16, mz16);
//}
//
//Vec3<float> BNO055::CompensateMag_(Vec3<int16_t> mag, uint16_t rhall) {
//    Vec3<float> result;
//    result.x = CompensateMagX_(mag.x >> 3, rhall);
//    result.y = CompensateMagY_(mag.y >> 3, rhall);
//    result.z = CompensateMagZ_(mag.z >> 1, rhall);
//    return result;
//}
//
///// https://github.com/boschsensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150_defs.h#L224
//constexpr uint8_t BMM150_DIG_X1       = 0x5D;
//constexpr uint8_t BMM150_DIG_Y1       = 0x5E;
//constexpr uint8_t BMM150_DIG_Z4_LSB   = 0x62;
//constexpr uint8_t BMM150_DIG_Z4_MSB   = 0x63;
//constexpr uint8_t BMM150_DIG_X2       = 0x64;
//constexpr uint8_t BMM150_DIG_Y2       = 0x65;
//constexpr uint8_t BMM150_DIG_Z2_LSB   = 0x68;
//constexpr uint8_t BMM150_DIG_Z2_MSB   = 0x69;
//constexpr uint8_t BMM150_DIG_Z1_LSB   = 0x6A;
//constexpr uint8_t BMM150_DIG_Z1_MSB   = 0x6B;
//constexpr uint8_t BMM150_DIG_XYZ1_LSB = 0x6C;
//constexpr uint8_t BMM150_DIG_XYZ1_MSB = 0x6D;
//constexpr uint8_t BMM150_DIG_Z3_LSB   = 0x6E;
//constexpr uint8_t BMM150_DIG_Z3_MSB   = 0x6F;
//constexpr uint8_t BMM150_DIG_XY2      = 0x70;
//constexpr uint8_t BMM150_DIG_XY1      = 0x71;
//
///*!
// * @brief This internal API reads the trim registers of the sensor and stores
// * the trim values in the "trim_data" of device structure.
// * @note BNO055は地磁気センサーとしてBMM150を搭載している
// * @note 参照:
// * https://github.com/boschsensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1199
// */
//void BNO055::ReadTrimMag_() {
//    uint8_t raw_trim_x1y1[2]     = {0};
//    uint8_t raw_trim_xyz_data[4] = {0};
//    uint8_t raw_trim_xy1xy2[10]  = {0};
//
//    // 一個一個読み出すと不安定な挙動を示したので、出典通りまとめて読み出す
//    WireHelper::readDataRaw(*wire_, MAGN_ADDRESS_, BMM150_DIG_X1, raw_trim_x1y1, sizeof(raw_trim_x1y1));
//    WireHelper::readDataRaw(*wire_, MAGN_ADDRESS_, BMM150_DIG_Z4_LSB, raw_trim_xyz_data, sizeof(raw_trim_xyz_data));
//    WireHelper::readDataRaw(*wire_, MAGN_ADDRESS_, BMM150_DIG_Z2_LSB, raw_trim_xy1xy2, sizeof(raw_trim_xy1xy2));
//
//    magTrimData_.dig_x1   = raw_trim_x1y1[0];
//    magTrimData_.dig_y1   = raw_trim_x1y1[1];
//    magTrimData_.dig_x2   = raw_trim_xyz_data[2];
//    magTrimData_.dig_y2   = raw_trim_xyz_data[3];
//    magTrimData_.dig_z1   = WireHelper::readDataFromRaw16(raw_trim_xy1xy2 + 2, Endianness::LITTLE);
//    magTrimData_.dig_z2   = WireHelper::readDataFromRaw16(raw_trim_xy1xy2, Endianness::LITTLE);
//    magTrimData_.dig_z3   = WireHelper::readDataFromRaw16(raw_trim_xy1xy2 + 6, Endianness::LITTLE);
//    magTrimData_.dig_z4   = WireHelper::readDataFromRaw16(raw_trim_xyz_data, Endianness::LITTLE);
//    magTrimData_.dig_xy1  = raw_trim_xy1xy2[9];
//    magTrimData_.dig_xy2  = raw_trim_xy1xy2[8];
//    magTrimData_.dig_xyz1 = WireHelper::readDataFromRaw16(raw_trim_xy1xy2 + 4, Endianness::LITTLE);
//}
//
//uint16_t BNO055::readRhall16_() {
//    uint16_t result = WireHelper::readData16(*wire_, MAGN_ADDRESS_, 0x48, Endianness::LITTLE);
//    return result >> 2;
//}
//
//constexpr int16_t BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP = -4096;
///*!
// * @brief This internal API is used to obtain the compensated
// * magnetometer x axis data(micro-tesla) in float.
// */
//// https://github.com/boschsensortec/BMM150-Sensor-API/blob/a20641f216057f0c54de115fe81b57368e119c01/bmm150.c#L1614
//float BNO055::CompensateMagX_(int16_t raw_mag_x, uint16_t rhall) {
//    float retval = 0;
//    float process_comp_x0;
//    float process_comp_x1;
//    float process_comp_x2;
//    float process_comp_x3;
//    float process_comp_x4;
//    /* Overflow condition check */
//    if ((raw_mag_x != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) && (rhall != 0) && (magTrimData_.dig_xyz1 != 0)) {
//        /* Processing compensation equations */
//        process_comp_x0 = (((float)magTrimData_.dig_xyz1) * 16384.0f / rhall);
//        retval          = (process_comp_x0 - 16384.0f);
//        process_comp_x1 = ((float)magTrimData_.dig_xy2) * (retval * retval / 268435456.0f);
//        process_comp_x2 = process_comp_x1 + retval * ((float)magTrimData_.dig_xy1) / 16384.0f;
//        process_comp_x3 = ((float)magTrimData_.dig_x2) + 160.0f;
//        process_comp_x4 = raw_mag_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
//        retval          = ((process_comp_x4 / 8192.0f) + (((float)magTrimData_.dig_x1) * 8.0f)) / 16.0f;
//    } else {
//        /* Overflow, set output to 0.0f */
//        retval = 0.0f;
//    }
//
//    return retval;
//}
//
//float BNO055::CompensateMagY_(int16_t raw_mag_y, uint16_t rhall) {
//    float retval = 0;
//    float process_comp_y0;
//    float process_comp_y1;
//    float process_comp_y2;
//    float process_comp_y3;
//    float process_comp_y4;
//
//    /* Overflow condition check */
//    if ((raw_mag_y != BMM150_OVERFLOW_ADCVAL_XYAXES_FLIP) && (rhall != 0) && (magTrimData_.dig_xyz1 != 0)) {
//        /* Processing compensation equations */
//        process_comp_y0 = ((float)magTrimData_.dig_xyz1) * 16384.0f / rhall;
//        retval          = process_comp_y0 - 16384.0f;
//        process_comp_y1 = ((float)magTrimData_.dig_xy2) * (retval * retval / 268435456.0f);
//        process_comp_y2 = process_comp_y1 + retval * ((float)magTrimData_.dig_xy1) / 16384.0f;
//        process_comp_y3 = ((float)magTrimData_.dig_y2) + 160.0f;
//        process_comp_y4 = raw_mag_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
//        retval          = ((process_comp_y4 / 8192.0f) + (((float)magTrimData_.dig_y1) * 8.0f)) / 16.0f;
//    } else {
//        /* Overflow, set output to 0.0f */
//        retval = 0.0f;
//    }
//
//    return retval;
//}
//
//constexpr int16_t BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL = -16384;
//float BNO055::CompensateMagZ_(int16_t raw_mag_z, uint16_t rhall) {
//    float retval = 0;
//    float process_comp_z0;
//    float process_comp_z1;
//    float process_comp_z2;
//    float process_comp_z3;
//    float process_comp_z4;
//    float process_comp_z5;
//
//    /* Overflow condition check */
//    if ((raw_mag_z != BMM150_OVERFLOW_ADCVAL_ZAXIS_HALL) && (magTrimData_.dig_z2 != 0) && (magTrimData_.dig_z1 != 0) &&
//        (magTrimData_.dig_xyz1 != 0) && (rhall != 0)) {
//        /* Processing compensation equations */
//        process_comp_z0 = ((float)raw_mag_z) - ((float)magTrimData_.dig_z4);
//        process_comp_z1 = ((float)rhall) - ((float)magTrimData_.dig_xyz1);
//        process_comp_z2 = (((float)magTrimData_.dig_z3) * process_comp_z1);
//        process_comp_z3 = ((float)magTrimData_.dig_z1) * ((float)rhall) / 32768.0f;
//        process_comp_z4 = ((float)magTrimData_.dig_z2) + process_comp_z3;
//        process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
//        retval          = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
//    } else {
//        /* Overflow, set output to 0.0f */
//        retval = 0.0f;
//    }
//    return retval;
//}
//
//float BNO055::G_PER_LSB_() const { return G_PER_LSB_(accelRange_); }
//
//float BNO055::G_PER_LSB_(AccelRange accelRange) {
//    // ±2g→0.98mg/LSB, ±4g→1.95ms/LSB, ±8g→3.91mg/LSB, ±16g→7.81mg/LSB
//    switch (accelRange) {
//        case AccelRange::PM2G:
//            return 0.98;
//        case AccelRange::PM4G:
//            return 1.95;
//        case AccelRange::PM8G:
//            return 3.91;
//        case AccelRange::PM16G:
//            return 7.81;
//        default:
//            return std::nan("");
//    }
//}
//float BNO055::LSB_PER_DPS_() const { return LSB_PER_DPS_(gyroRange_); }
//float BNO055::LSB_PER_DPS_(GyroRange gyroRange) {
//    // ±2000°/s→16.4LSB/°/s, ±1000°/s→32.8LSB/°/s, ±500°/s→65.6LSB/°/s, ±250°/s→131.2LSB/°/s, ±125°/s→262.4LSB/°/s
//    switch (gyroRange) {
//        case GyroRange::PM2000DPS:
//            return 16.4;
//        case GyroRange::PM1000DPS:
//            return 32.8;
//        case GyroRange::PM500DPS:
//            return 65.6;
//        case GyroRange::PM250DPS:
//            return 131.2;
//        case GyroRange::PM125DPS:
//            return 262.4;
//        default:
//            return std::nan("");
//    }
//}
//float BNO055::DPS_PER_LSB_() const { return DPS_PER_LSB_(gyroRange_); }
//float BNO055::DPS_PER_LSB_(GyroRange gyroRange) { return 1 / LSB_PER_DPS_(gyroRange); }
//float BNO055::uT_PER_LSB_() const { return 1 / 16.0; }
//
//void BNO055::setAccelAddress(uint8_t addr) { ACCL_ADDRESS_ = addr; }
//void BNO055::setGyroAddress(uint8_t addr) { GYRO_ADDRESS_ = addr; }
//void BNO055::setMagnAddress(uint8_t addr) { MAGN_ADDRESS_ = addr; }
//uint8_t BNO055::accelAddress() const noexcept { return ACCL_ADDRESS_; }
//uint8_t BNO055::gyroAddress() const noexcept { return GYRO_ADDRESS_; }
//uint8_t BNO055::magnAddress() const noexcept { return MAGN_ADDRESS_; }
//int BNO055::absAccelMax() {
//    switch (accelRange_) {
//        case AccelRange::PM2G:
//            return 2;
//        case AccelRange::PM4G:
//            return 4;
//        case AccelRange::PM8G:
//            return 8;
//        case AccelRange::PM16G:
//            return 16;
//        default:
//            return 0;
//    }
//}
//int BNO055::absGyroMax() {
//    switch (gyroRange_) {
//        case GyroRange::PM125DPS:
//            return 125;
//        case GyroRange::PM250DPS:
//            return 250;
//        case GyroRange::PM500DPS:
//            return 500;
//        case GyroRange::PM1000DPS:
//            return 1000;
//        case GyroRange::PM2000DPS:
//            return 2000;
//        default:
//            return 0;
//    }
//}
//BNO055::AccelRange BNO055::accelRangeFromAbsAccelMax(int absAccelMax) {
//    switch (absAccelMax) {
//        case 2:
//            return AccelRange::PM2G;
//        case 4:
//            return AccelRange::PM4G;
//        case 8:
//            return AccelRange::PM8G;
//        case 16:
//            return AccelRange::PM16G;
//        default:
//            return AccelRange::INVALID;
//    }
//}
//BNO055::GyroRange BNO055::gyroRangeFromAbsGyroMax(int absGyroMax) {
//    switch (absGyroMax) {
//        case 125:
//            return GyroRange::PM125DPS;
//        case 250:
//            return GyroRange::PM250DPS;
//        case 500:
//            return GyroRange::PM500DPS;
//        case 1000:
//            return GyroRange::PM1000DPS;
//        case 2000:
//            return GyroRange::PM2000DPS;
//        default:
//            return GyroRange::INVALID;
//    }
//}
//Vec3<float> BNO055::readAccel() { return convertAccelFrom16(readAccel16_(), accelRange_); }
//Vec3<float> BNO055::readGyro() { return convertGyroFrom16(readGyro16_(), gyroRange_); }
//Vec3<float> BNO055::readMag() { return CompensateMag_(readMag16_(), readRhall16_()); }
//}  // namespace Lightus::Sensors
//
//#if !defined(LIGHTUS_SENSORS_NO_BNO055_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
//Lightus::Sensors::BNO055 BNO055;
//#endif
