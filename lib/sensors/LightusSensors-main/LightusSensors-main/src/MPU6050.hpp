#ifndef LIGHTUS_SENSOR_MPU6050_HPP
#define LIGHTUS_SENSOR_MPU6050_HPP
#include <ArduinoJson.h>
#include <Wire.h>

#include <cstdint>

#include "IMUData.hpp"
namespace Lightus::Sensors {
class MPU6050 {
   public:
    enum class AccelRange : uint8_t {
        PM2G  = 0,  // ±2g
        PM4G  = 1,
        PM8G  = 2,
        PM16G = 3,
        INVALID,
    };
    enum class GyroRange : uint8_t {
        PM2000DPS = 3,  // ±2000°/s
        PM1000DPS = 2,
        PM500DPS  = 1,
        PM250DPS  = 0,
        INVALID,
    };

    /// @brief MPU6050を起動する
    /// @param accelRange 取得する加速度の範囲。大きいほど精度は粗くなる
    /// @param gyroRange 取得する角速度の範囲。大きいほど精度は粗くなる
    /// @warning この関数を呼ぶ前にWire.begin()すること！！！
    void begin(TwoWire& wire, AccelRange accelRange = AccelRange::PM2G, GyroRange gyroRange = GyroRange::PM500DPS);

    using DataType = IMUData<6, float>;
    /// @brief MPU6050からデータを読み出す
    /// @return 6軸データ
    DataType read();
    using RawDataType = IMUData<6, int16_t>;
    RawDataType read16();

    String readCompensationParametersJSON();
    void dumpCompensationParametersToJSON(JsonObject obj);

    // 個別に各データを取り出す

    Vec3<float> readAccel();
    Vec3<int16_t> readAccel16();

    Vec3<float> readGyro();
    Vec3<int16_t> readGyro16();

    static Vec3<float> convertAccelFrom16(Vec3<int16_t> accel16, AccelRange range = AccelRange::PM2G);
    static Vec3<float> convertGyroFrom16(Vec3<int16_t> gyro16, GyroRange range = GyroRange::PM500DPS);

    void setI2CAddress(uint8_t addr);
    uint8_t I2CAddress() const noexcept;

    /// @brief 加速度の絶対値の最大値
    /// @return 最大値
    int absAccelMax();
    int absGyroMax();

    AccelRange accelRangeFromAbsAccelMax(int absAccelMax);
    GyroRange gyroRangeFromAbsGyroMax(int absGyroMax);

   private:
    TwoWire* wire_;
    AccelRange accelRange_;
    GyroRange gyroRange_;
    uint8_t I2C_ADDR = 0b1101000;  // AD0==LOW -> 0b1101000; AD0==HIGH -> 0b1101001
    Vec3<int16_t> readAccel16_();
    Vec3<int16_t> readGyro16_();

    void dumpAccelCompensationParametersToJson_(JsonObject obj);
    void dumpGyroCompensationParametersToJson_(JsonObject obj);

    int LSB_PER_G_() const;
    float G_PER_LSB_() const;
    float LSB_PER_DPS_() const;
    float DPS_PER_LSB_() const;

    static int LSB_PER_G_(AccelRange range);
    static float G_PER_LSB_(AccelRange range);
    static float LSB_PER_DPS_(GyroRange range);
    static float DPS_PER_LSB_(GyroRange range);
};
}  // namespace Lightus::Sensors
#if !defined(LIGHTUS_SENSORS_NO_MPU6050_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
extern Lightus::Sensors::MPU6050 mpu6050;
#endif
#endif