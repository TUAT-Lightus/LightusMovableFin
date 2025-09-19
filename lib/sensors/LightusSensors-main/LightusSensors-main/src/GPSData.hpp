#ifndef LIGHTUS_SENSORS_GPSDATA_HPP
#define LIGHTUS_SENSORS_GPSDATA_HPP
#include <Arduino.h>
namespace Lightus::Sensors {
// TinyGPS++ではさらにlocation,altitude,timeに細分化されているが、今(加太2024前)は割愛
struct GPSData {
    double latitude;   // 緯度
    double longitude;  // 経度
    double altitude;   // 高度
    long h;            // GPS時
    long m;            // GPS分
    long s;            // GPS秒
    long ms;           // GPSミリ秒

    String toString();
    String toCSVString();
    static String CSVHeader();
    String toTeleplotString();
};
}  // namespace Lightus::Sensors
#endif