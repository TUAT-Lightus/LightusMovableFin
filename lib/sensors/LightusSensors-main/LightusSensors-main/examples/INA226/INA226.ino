#include <Wire.h>
#include <INA226.hpp>

uint32_t currentTime, loopTimer;
uint32_t loopTime_flight = 4000;                       // データレート[us]
uint16_t dataRate        = 1000000 / loopTime_flight;  // データレート[Hz]

#define SDA 1
#define SCL 2


void setup() {
    Serial.begin(115200);

#ifdef ARDUINO_ARCH_ESP32
    Wire.setPins(SDA, SCL);  // ESP32用のWireライブラリだとこうやって指定するらしい
#else
    Wire.setSDA(SDA);
    Wire.setSCL(SCL);
#endif
    Wire.begin();
    Wire.setClock(1000000);

    ina226.begin(Wire);
    Serial.println(Lightus::Sensors::INA226::DataType::CSVHeader());
}

void loop() {
    auto ina226Data = ina226.busVoltage(0x40);
    Serial.println(ina226Data.toCSVString());

    // ループ速度管理
    while (currentTime - loopTimer <= loopTime_flight) {
        currentTime = micros();
    }
    loopTimer = currentTime;
}