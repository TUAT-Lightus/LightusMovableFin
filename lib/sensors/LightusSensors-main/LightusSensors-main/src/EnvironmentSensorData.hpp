#ifndef LIGHTUS_SENSORS_ENVIRONMENT_HPP
#define LIGHTUS_SENSORS_ENVIRONMENT_HPP
#include <Arduino.h>

namespace Lightus::Sensors {
template <class T>
struct EnvironmentSensorData {
    T temperature = 0;
    T pressure    = 0;
    T humidity    = 0;

    String toString() {
        return String("{temperature: ") + String(this->temperature) + String(", pressure: ") + String(this->pressure) +
               String(", humidity: ") + String(this->humidity) + String("}");
    }
    String toCSVString() {
        return String(this->temperature) + String(", ") + String(this->pressure) + String(", ") +
               String(this->humidity);
    }
    static String CSVHeader() { return String("temperature, pressure, humidity"); }
    String toTeleplotString() {
        String result;
        result += ">" + String("temperature:") + String(temperature) + "\n";
        result += ">" + String("pressure:") + String(pressure) + "\n";
        result += ">" + String("humidity:") + String(humidity) + "\n";
        return result;
    }
};
}  // namespace Lightus::Sensors

#endif  // LIGHTUS_SENSORS_ENVIRONMENT_HPP
