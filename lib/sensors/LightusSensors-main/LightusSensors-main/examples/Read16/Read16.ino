#include <Wire.h>

#include <BMX055.hpp>

uint32_t currentTime, loopTimer;
uint32_t loopTime_flight = 4000;                       // データレート[us]
uint16_t dataRate        = 1000000 / loopTime_flight;  // データレート[Hz]

namespace Pins {
constexpr int I2C_SDA = 10;
constexpr int I2C_SCL = 11;
}  // namespace Pins

void setup() {
    Serial.begin(115200);

#ifdef ARDUINO_ARCH_ESP32
    Wire.setPins(I2C_SDA, I2C_SCL);  // ESP32用のWireライブラリだとこうやって指定するらしい
#else
    Wire.setSDA(I2C_SDA);
    Wire.setSCL(I2C_SCL);
#endif
    Wire.begin();
    Wire.setClock(1000000);

    bmx055.begin(Wire, Lightus::Sensors::BMX055::AccelRange::PM16G, Lightus::Sensors::BMX055::GyroRange::PM2000DPS);
    Serial.println(Lightus::Sensors::BMX055::DataType::CSVHeader());
}

void loop() {
    auto accel16 = bmx055.readAccel16();
    auto gyro16  = bmx055.readGyro16();

    auto accelFloat =
        Lightus::Sensors::BMX055::convertAccelFrom16(accel16, Lightus::Sensors::BMX055::AccelRange::PM16G);
    auto gyroFloat =
        Lightus::Sensors::BMX055::convertGyroFrom16(gyro16, Lightus::Sensors::BMX055::GyroRange::PM2000DPS);
    Serial.println(accelFloat.toCSVString() + String(", ") + gyroFloat.toCSVString());

    // ループ速度管理
    while (currentTime - loopTimer <= loopTime_flight) {
        currentTime = micros();
    }
    loopTimer = currentTime;
}