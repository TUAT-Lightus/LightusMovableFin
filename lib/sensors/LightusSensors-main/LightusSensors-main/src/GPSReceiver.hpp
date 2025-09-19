#ifndef LIGHTUS_SENSORS_TINYGPSPLUSPLUS_HPP
#define LIGHTUS_SENSORS_TINYGPSPLUSPLUS_HPP
#include <Arduino.h>
#include <TinyGPS++.h>

#include "GPSData.hpp"
namespace Lightus::Sensors {
class GPSReceiver {
   public:
    using DataType = GPSData;
    void begin(Stream& serial);
    void update();
    bool available();
    GPSData read();

   private:
    TinyGPSPlus gps_;
    Stream* GPSSerial_;
};
}  // namespace Lightus::Sensors
#if !defined(LIGHTUS_SENSORS_NO_TINYGPSPLUSPLUS_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GPSRECEIVER_INSTANCE) && \
    !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
extern Lightus::Sensors::GPSReceiver gps;
#endif
#endif