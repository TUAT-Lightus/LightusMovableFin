#include <Wire.h>

#include <NTCThermistor.hpp>

uint32_t currentTime, loopTimer;
uint32_t loopTime_flight = 4000;                       // データレート[us]
uint16_t dataRate        = 1000000 / loopTime_flight;  // データレート[Hz]

// 103JT-025
// https://akizukidenshi.com/catalog/g/g110160/
// https://akizukidenshi.com/goodsaffix/jt_thermistor.pdf
constexpr float B  = 3435;
constexpr float R0 = 10'000;

constexpr float R1 = 100'000;

using Lightus::Sensors::NTCThermistor;
NTCThermistor therm0;
NTCThermistor therm1;
NTCThermistor therm2;
NTCThermistor therm3;
NTCThermistor therm4;

template <typename T>
using Celsius = Lightus::Sensors::Celsius<T>;

void setup() {
    Serial.begin(115200);
    therm0.begin(6, B, R0, R1);
    therm1.begin(7, B, R0, R1);
    therm2.begin(8, B, R0, R1);
    therm3.begin(9, B, R0, R1);
    therm4.begin(10, B, R0, R1);
}

void loop() {
    Serial.print(static_cast<Celsius<float>>(therm0.read()).value);
    Serial.print(",");
    Serial.print(static_cast<Celsius<float>>(therm1.read()).value);
    Serial.print(",");
    Serial.print(static_cast<Celsius<float>>(therm2.read()).value);
    Serial.print(",");
    Serial.print(static_cast<Celsius<float>>(therm3.read()).value);
    Serial.print(",");
    Serial.print(static_cast<Celsius<float>>(therm4.read()).value);
    Serial.println();

    // ループ速度管理
    while (currentTime - loopTimer <= loopTime_flight) {
        currentTime = micros();
    }
    loopTimer = currentTime;
}
