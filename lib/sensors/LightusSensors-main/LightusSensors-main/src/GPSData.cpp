#include "GPSData.hpp"
namespace Lightus::Sensors {
String GPSData::toString() {
    return String("{ latitude: ") + String(latitude, 5) + String(", longitude: ") + String(longitude, 5) +
           String(", altitude: ") + String(altitude, 5) + String(", h: ") + String(h) + String(", m: ") + String(m) +
           String(", s: ") + String(s) + String(", ms: ") + String(ms) + String("}");
}
String GPSData::toCSVString() {
    return String(latitude, 5) + String(", ") + String(longitude, 5) + String(", ") + String(altitude, 5) +
           String(", ") + String(h) + String(", ") + String(m) + String(", ") + String(s) + String(", ") + String(ms);
}
String GPSData::CSVHeader() { return {"latitude, longitude, altitude, h, m, s, ms"}; }
String GPSData::toTeleplotString() {
    return String(">latitude: ") + String(latitude, 5) + String("\n>longitude: ") + String(longitude, 5) +
           String("\n>altitude: ") + String(altitude, 5) + String("\n>h: ") + String(h) + String("\n>m: ") + String(m) +
           String("\n>s: ") + String(s) + String("\n>ms: ") + String(ms) + String("\n");
}
}  // namespace Lightus::Sensors