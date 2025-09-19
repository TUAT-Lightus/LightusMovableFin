#ifndef LIGHTUS_SENSOR_IMU_HPP
#define LIGHTUS_SENSOR_IMU_HPP
#include "Vec3.hpp"
namespace Lightus::Sensors {
template <int DOF, class T>
struct IMUData;

template <class T>
struct IMUData<3, T> {
    Vec3<T> accel;

    String toString() const { return String("{ accel: ") + this->accel.toString() + String("}"); }
    String toCSVString() const { return this->accel.toCSVString(); }
    static String CSVHeader() { return "ax, ay, az"; }
    String toTeleplotString() const { return accel.toTeleplotString("accel_"); }
};

template <class T>
struct IMUData<6, T> {
    Vec3<T> accel;
    Vec3<T> gyro;

    String toString() const {
        return String("{ accel: ") + this->accel.toString() + String(", gyro: ") + this->gyro.toString() + String("}");
    }
    String toCSVString() const { return this->accel.toCSVString() + String(", ") + this->gyro.toCSVString(); }
    static String CSVHeader() { return "ax, ay, az, gx, gy, gz"; }
    String toTeleplotString() const { return accel.toTeleplotString("accel_") + gyro.toTeleplotString("gyro_"); }
};

template <class T>
struct IMUData<9, T> {
    Vec3<T> accel;
    Vec3<T> gyro;
    Vec3<T> mag;

    String toString() const {
        return String("{ accel: ") + this->accel.toString() + String(", gyro: ") + this->gyro.toString() +
               String(", mag: ") + this->mag.toString() + String("}");
    }
    String toCSVString() const {
        return this->accel.toCSVString() + String(", ") + this->gyro.toCSVString() + String(", ") +
               this->mag.toCSVString();
    }
    static String CSVHeader() { return "ax, ay, az, gx, gy, gz, mx, my, mz"; }
    String toTeleplotString() const {
        return accel.toTeleplotString("accel_") + gyro.toTeleplotString("gyro_") + mag.toTeleplotString("mag_");
    }
};
}  // namespace Lightus::Sensors
#endif