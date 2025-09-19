#ifndef LIGHTUS_SENSORS_VECTOR_HPP
#define LIGHTUS_SENSORS_VECTOR_HPP
#include <Arduino.h>

#include <cmath>
#include <type_traits>
namespace Lightus {
// 演算子の定義に便利な型エイリアスたち

// T型の値をU型の値でシフトした結果この型になる
template <typename T, typename U>
using RightShifted = decltype(std::declval<T>() >> std::declval<U>());
// T型の値をU型の値でシフトした結果この型になる
template <typename T, typename U>
using LeftShifted = decltype(std::declval<T>() << std::declval<U>());
// T型の値とU型の値を掛け算した結果この型になる
template <typename T, typename U>
using Multiplied = decltype(std::declval<T>() * std::declval<U>());
// T型の値をU型の値で割った結果この型になる
template <typename T, typename U>
using Divided = decltype(std::declval<T>() / std::declval<U>());
// T型の値とU型の値を足し合わせた結果この型になる
template <typename T, typename U>
using Added = decltype(std::declval<T>() + std::declval<U>());
// T型の値からU型の値を引いた結果この型になる
template <typename T, typename U>
using Subtracted = decltype(std::declval<T>() - std::declval<U>());

/// @brief 3次元ベクトル
/// @tparam T 各成分の型
template <typename T>
struct Vec3 {
    T x, y, z;
    Vec3() : Vec3(0, 0, 0) {}
    Vec3(T x, T y, T z) : x(x), y(y), z(z) {}
    Vec3<T> &operator=(const Vec3<T> &theother);

    /// @brief ベクトルの中身の型を変える
    /// @tparam U 目的の型
    template <typename U>
    operator Vec3<U>();

    /// @brief ベクトルから人間が読みやすい形式の文字列を生成する
    /// @return 人間が読みやすい形式の文字列
    String toString() const;

    /// @brief ベクトルからCSV形式の文字列を生成する
    /// @return CSV形式の文字列
    String toCSVString() const;

    /// @brief ベクトルの長さを計算する
    /// @return ベクトルの長さ
    T norm();

    /// @brief Teleplot形式の文字列を生成する
    /// @param prefix 系列名の前に付ける文字列。
    /// @param capibal x,y,zの文字を大文字にするか否か
    /// @return
    String toTeleplotString(String prefix, bool capital = false) const;

    template <typename U>
    Vec3<T> &operator<<=(const U a);
    template <typename U>
    Vec3<T> &operator>>=(const U a);
    template <typename U>
    Vec3<T> &operator*=(const U a);
    template <typename U>
    Vec3<T> &operator/=(const U a);
    template <typename U>
    Vec3<T> &operator+=(const Vec3<U> &theother);
    template <typename U>
    Vec3<T> &operator-=(const Vec3<U> &theother);
};

// テンプレートが使われているので、以下に実装を置く

template <typename T>
Vec3<T> &Vec3<T>::operator=(const Vec3<T> &theother) {
    this->x = theother.x;
    this->y = theother.y;
    this->z = theother.z;
    return *this;
}

template <typename T>
template <typename U>
Vec3<T>::operator Vec3<U>() {
    return Vec3<U>(static_cast<U>(this->x), static_cast<U>(this->y), static_cast<U>(this->z));
}

template <typename T>
String Vec3<T>::toString() const {
    return String("(") + String(this->x) + String(", ") + String(this->y) + String(", ") + String(this->z) +
           String(")");
}

template <typename T>
String Vec3<T>::toCSVString() const {
    return String(this->x) + String(", ") + String(this->y) + String(", ") + String(this->z);
}

template <typename T, typename U>
Vec3<RightShifted<T, U>> operator>>(const Vec3<T> &target, const U a) {
    return {target.x >> a, target.y >> a, target.z >> a};
}

template <typename T, typename U>
Vec3<LeftShifted<T, U>> operator<<(const Vec3<T> &target, const U a) {
    return {target.x << a, target.y << a, target.z << a};
}

template <typename T, typename U>
Vec3<Multiplied<T, U>> operator*(const Vec3<T> &target, const U a) {
    return {target.x * a, target.y * a, target.z * a};
}
template <typename T, typename U>
Vec3<Multiplied<T, U>> operator*(const U a, const Vec3<T> &target) {
    return target * a;
}

template <typename T, typename U>
Vec3<Divided<T, U>> operator/(const Vec3<T> &target, U a) {
    return {target.x / a, target.y / a, target.z / a};
}

template <typename T, typename U>
Vec3<Added<T, U>> operator+(const Vec3<T> &one, const Vec3<U> &theother) {
    return {one.x + theother.x, one.y + theother.y, one.z + theother.z};
}

template <typename T, typename U>
Vec3<Subtracted<T, U>> operator-(const Vec3<T> &one, const Vec3<U> &theother) {
    return {one.x - theother.x, one.y - theother.y, one.z - theother.z};
}

template <typename T>
T Vec3<T>::norm() {
    return std::sqrt(x * x + y * y + z * z);
}
template <typename T>
String Vec3<T>::toTeleplotString(String prefix, bool capital) const {
    String result;
    result += ">" + prefix + (capital ? "X" : "x") + String(":") + String(x) + "\n";
    result += ">" + prefix + (capital ? "Y" : "y") + String(":") + String(y) + "\n";
    result += ">" + prefix + (capital ? "Z" : "z") + String(":") + String(z) + "\n";
    return result;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator<<=(const U a) {
    *this = (*this) << a;
    return *this;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator>>=(const U a) {
    *this = (*this) >> a;
    return *this;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator*=(const U a) {
    *this = (*this) * a;
    return *this;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator/=(const U a) {
    *this = (*this) / a;
    return *this;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator+=(const Vec3<U> &theother) {
    *this = (*this) + theother;
    return *this;
}
template <typename T>
template <typename U>
inline Vec3<T> &Vec3<T>::operator-=(const Vec3<U> &theother) {
    *this = (*this) - theother;
    return *this;
}
}  // namespace Lightus
#endif  // LIGHTUS_SENSORS_VECTOR_HPP
