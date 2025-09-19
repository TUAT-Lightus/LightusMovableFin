#include <Wire.h>

#include <BME280.hpp>
#include <BMX055.hpp>
#include <GPSReceiver.hpp>

uint32_t currentTime, loopTimer;
uint32_t loopTime_flight = 4000;                       // データレート[us]
uint16_t dataRate        = 1000000 / loopTime_flight;  // データレート[Hz]

namespace Pins {
constexpr int I2C_SDA = 10;
constexpr int I2C_SCL = 11;
}  // namespace Pins
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);

#ifdef ARDUINO_ARCH_ESP32
    Wire.setPins(Pins::I2C_SDA, Pins::I2C_SCL);  // ESP32用のWireライブラリだとこうやって指定するらしい
#else
    Wire.setSDA(Pins::I2C_SDA);
    Wire.setSCL(Pins::I2C_SCL);
#endif
    Wire.begin();
    Wire.setClock(1000000);

    bmx055.begin(Wire);
    bme280.begin(Wire);
    gps.begin(Serial2);
    Serial.println(Lightus::Sensors::BMX055::DataType::CSVHeader() + String(", ") +
                   Lightus::Sensors::BME280::DataType::CSVHeader() + String(", ") +
                   Lightus::Sensors::GPSReceiver::DataType::CSVHeader());
}

void loop() {
    auto bmxData = bmx055.read();
    auto bmeData = bme280.read();
    Serial.print(bmxData.toCSVString() + String(", ") + bmeData.toCSVString());
    gps.update();           // gpsのデータをシリアルから読み取り更新
    if (gps.available()) {  // fixするとtrueが返ってくる
        auto gpsData = gps.read();
        Serial.print(String(", ") + gpsData.toCSVString());
    } else {
        Serial.print("gps not fixed");
    }
    Serial.println("");

    // ループ速度管理
    while (currentTime - loopTimer <= loopTime_flight) {
        currentTime = micros();
    }
    loopTimer = currentTime;
}
