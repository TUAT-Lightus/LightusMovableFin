//#ifndef LIGHTUS_SENSOR_BNO055_HPP
//#define LIGHTUS_SENSOR_BNO055_HPP
//#include <ArduinoJson.h>
//#include <Wire.h>
//
//#include <cstdint>
//
//#include "IMUData.hpp"
//namespace Lightus::Sensors {
//class BNO055 {
//   public:
//    enum class AccelRange {
//        PM2G  = 0b0011,  // ±2g
//        PM4G  = 0b0101,
//        PM8G  = 0b1000,
//        PM16G = 0b1100,
//        INVALID,
//    };
//    enum class GyroRange {
//        PM2000DPS = 0b000,  // ±2000°/s
//        PM1000DPS = 0b001,
//        PM500DPS  = 0b010,
//        PM250DPS  = 0b011,
//        PM125DPS  = 0b100,
//        INVALID,
//    };
//    enum class MagneticRate {
//        Hz10 = 0b000,  // 10Hz
//        Hz2  = 0b001,
//        Hz6  = 0b010,
//        Hz15 = 0b100,
//        Hz20 = 0b101,
//        Hz25 = 0b110,
//        Hz30 = 0b111,
//        INVALID,
//    };
//    enum class FilterConfig { FILTERED = 0, UNFILTERED = 1, INVALID };
//    /// @brief BNO055を起動する
//    /// @param accelRange 取得する加速度の範囲。大きいほど精度は粗くなる
//    /// @param gyroRange 取得する角速度の範囲。大きいほど精度は粗くなる
//    /// @param magneticRate 地磁気センサーのデータレート？
//    /// @warning この関数を呼ぶ前にWire.begin()すること！！！
//    void begin(TwoWire& wire, AccelRange accelRange = AccelRange::PM2G, GyroRange gyroRange = GyroRange::PM500DPS,
//               MagneticRate magneticRate = MagneticRate::Hz10, FilterConfig filter = FilterConfig::FILTERED);
//
//    using DataType = IMUData<9, float>;
//    /// @brief BNO055からデータを読み出す
//    /// @return 9軸データ
//    DataType read();
//    using RawDataType = IMUData<9, int16_t>;
//    RawDataType read16();
//
//    String readCompensationParametersJSON();
//    void dumpCompensationParametersToJSON(JsonObject obj);
//
//    // 個別に各データを取り出す
//
//    Vec3<float> readAccel();
//    Vec3<int16_t> readAccel16();
//
//    Vec3<float> readGyro();
//    Vec3<int16_t> readGyro16();
//
//    Vec3<float> readMag();
//    Vec3<int16_t> readMag16();
//    // 地磁気センサーからの値はセンサーから読み取った補正データを用いて補正する必要があるため、単独で取り出しても意味が薄い
//
//    static Vec3<float> convertAccelFrom16(Vec3<int16_t> accel16, AccelRange range = AccelRange::PM2G);
//    static Vec3<float> convertGyroFrom16(Vec3<int16_t> gyro16, GyroRange range = GyroRange::PM500DPS);
//
//    void setAccelAddress(uint8_t addr);
//    void setGyroAddress(uint8_t addr);
//    void setMagnAddress(uint8_t addr);
//
//    uint8_t accelAddress() const noexcept;
//    uint8_t gyroAddress() const noexcept;
//    uint8_t magnAddress() const noexcept;
//
//    /// @brief 加速度の絶対値の最大値
//    /// @return 最大値
//    int absAccelMax();
//    int absGyroMax();
//
//    AccelRange accelRangeFromAbsAccelMax(int absAccelMax);
//    GyroRange gyroRangeFromAbsGyroMax(int absGyroMax);
//
//   private:
//    TwoWire* wire_;
//    AccelRange accelRange_;
//    GyroRange gyroRange_;
//    MagneticRate magneticRate_;
//    uint8_t BNO_ADDRESS_ = 0x28;
//    //uint8_t GYRO_ADDRESS_ = 0x69;
//    //uint8_t MAGN_ADDRESS_ = 0x13;
//    Vec3<int16_t> readAccel16_();
//    Vec3<int16_t> readGyro16_();
//    Vec3<int16_t> readMag16_();
//
//    struct BMM150TrimData {
//        uint8_t dig_x1, dig_y1, dig_x2, dig_y2;
//        uint16_t dig_z1, dig_z2, dig_z3, dig_z4;
//        uint8_t dig_xy1, dig_xy2;
//        uint16_t dig_xyz1;
//        void toJSON(JsonObject obj);
//    } magTrimData_;
//
//    void dumpAccelCompensationParametersToJson_(JsonObject obj);
//    void dumpGyroCompensationParametersToJson_(JsonObject obj);
//    void dumpMagCompensationParametersToJson_(JsonObject obj);
//
//    void ReadTrimMag_();
//
//    uint16_t readRhall16_();
//
//    Vec3<float> CompensateMag_(Vec3<int16_t> mag, uint16_t rhall);
//    float CompensateMagX_(int16_t raw_mag_x, uint16_t rhall);
//    float CompensateMagY_(int16_t raw_mag_y, uint16_t rhall);
//    float CompensateMagZ_(int16_t raw_mag_z, uint16_t rhall);
//
//    float G_PER_LSB_() const;
//    float LSB_PER_DPS_() const;
//    float DPS_PER_LSB_() const;
//    float uT_PER_LSB_() const;
//
//    static float G_PER_LSB_(AccelRange range);
//    static float LSB_PER_DPS_(GyroRange range);
//    static float DPS_PER_LSB_(GyroRange range);
//};
//}  // namespace Lightus::Sensors
//#if !defined(LIGHTUS_SENSORS_NO_BNO055_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
//extern Lightus::Sensors::BNO055 BNO055;
//#endif
//#endif
