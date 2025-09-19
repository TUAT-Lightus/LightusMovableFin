#ifndef LIGHTUS_SENSORS_TEMPEARTUREDATA
#define LIGHTUS_SENSORS_TEMPEARTUREDATA
namespace Lightus::Sensors {

template <typename T>
struct Kelvin {
    T value;

    template <typename U>
    operator Kelvin<U>() const {
        return {static_cast<U>(value)};
    }
};

template <typename T>
struct Celsius {
    T value;

    static constexpr float KelvinOffset = 273.15;

    template <typename U>
    operator Celsius<U>() const {
        return {static_cast<U>(value)};
    }

    template <typename U>
    explicit operator Kelvin<U>() const {
        return {value + KelvinOffset};
    }

    template <typename U>
    explicit Celsius(const Kelvin<U>& kelvin) {
        this->value = kelvin.value - KelvinOffset;
    }

    Celsius() = default;
    Celsius(T value) : value(value) {}
};

inline namespace literals {
inline Celsius<long double> operator"" _c(long double deg) { return {deg}; }
inline Kelvin<long double> operator"" _k(long double kel) { return {kel}; }
};  // namespace literals

}  // namespace Lightus::Sensors
#endif
