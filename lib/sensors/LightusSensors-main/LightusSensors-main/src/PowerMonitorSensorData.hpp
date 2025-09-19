#ifndef LIGHTUS_SENSORS_POWER_MONITOR_SENSOR_DATA_HPP
#define LIGHTUS_SENSORS_POWER_MONITOR_SENSOR_DATA_HPP
#include <Arduino.h>

#include <serial_utils.hpp>
namespace Lightus::Sensors {
template <class T>
class PowerMonitorSensorData {
   public:
    T voltage = 0;
    T current = 0;

    String toString() {
        Lightus::StringStream sstream;
        Lightus::StreamWithName stream;
        stream.begin(sstream);
        stream.multiPrintWithName(withName(voltage), withName(current));
        return sstream.readAll();
    }
    String toCSVString() {
        Lightus::StringStream sstream;
        Lightus::StreamWithName stream;
        stream.begin(sstream);
        stream.setPreset(Lightus::StreamWithName::Preset::CSV);
        stream.multiPrintWithName(withoutName(voltage), withoutName(current));
        return sstream.readAll();
    }
    static String CSVHeader() { return String("voltage, current"); }
    String toTeleplotString() {
        Lightus::StringStream sstream;
        Lightus::StreamWithName stream;
        stream.begin(sstream);
        stream.setPreset(Lightus::StreamWithName::Preset::TELEPLOT);
        stream.multiPrintlnWithName(withName(voltage), withName(current));
        return sstream.readAll();
    }
};
}  // namespace Lightus::Sensors
#endif