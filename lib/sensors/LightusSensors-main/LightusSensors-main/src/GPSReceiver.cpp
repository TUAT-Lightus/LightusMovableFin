#include "GPSReceiver.hpp"
namespace Lightus::Sensors {
void GPSReceiver::begin(Stream& serial) { GPSSerial_ = &serial; }
void GPSReceiver::update() {
    while (GPSSerial_->available()) {
        gps_.encode(GPSSerial_->read());
    }
}
bool GPSReceiver::available() { return gps_.location.isUpdated(); }
GPSData GPSReceiver::read() {
    GPSData result;
    // データ処理
    // GPS_データの照会
    result.latitude  = gps_.location.lat();
    result.longitude = gps_.location.lng();
    result.altitude  = gps_.altitude.meters();
    result.h         = gps_.time.hour();
    result.m         = gps_.time.minute();
    result.s         = gps_.time.second();
    result.ms        = gps_.time.centisecond();
    return result;
}
}  // namespace Lightus::Sensors
#if !defined(LIGHTUS_SENSORS_NO_TINYGPSPLUSPLUS_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GPSRECEIVER_INSTANCE) && \
    !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
Lightus::Sensors::GPSReceiver gps;
#endif