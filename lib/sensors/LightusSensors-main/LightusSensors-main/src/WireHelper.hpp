#ifndef LIGHTUS_SENSORS_WIREHELPER_HPP
#define LIGHTUS_SENSORS_WIREHELPER_HPP

#include <Wire.h>

#include <cstddef>
#include <cstdint>
namespace Lightus::WireHelper {
enum class Endianness { BIG, LITTLE };
void writeTo(TwoWire& wire, uint16_t address, uint16_t val);          // slaveに値を送るだけ
void setVal(TwoWire& wire, uint16_t add, uint16_t reg, uint8_t val);  // 任意のレジスタに値をセット
void setVal16(TwoWire& wire, uint16_t add, uint16_t reg, uint16_t val,
              Endianness endian);  // 任意のレジスタに値をセット
void setVal24(TwoWire& wire, uint16_t add, uint16_t reg, uint32_t val,
              Endianness endian);  // 任意のレジスタに値をセット
void setValRaw(TwoWire& wire, uint16_t add, uint16_t reg, uint8_t* data, size_t len);
uint8_t readData8(TwoWire& wire, uint16_t address1, uint16_t address2);
uint16_t readData16(TwoWire& wire, uint16_t address1, uint16_t address2, Endianness endian);
uint32_t readData24(TwoWire& wire, uint16_t address1, uint16_t address2, Endianness endian);
/// @brief 任意の長さのデータを読み出す
/// @param address1 読み出すデバイスのI2C上のアドレス
/// @param address2 読み出すデバイス内部のレジスタのアドレス
/// @param[out] dst 出力先
/// @param len 読み出すデータのバイト長
/// @param timeout 読出しがタイムアウトするまでの時間[ms] -1を指定すると事実上タイムアウトなしになる
/// @return 読み出せたバイト長
size_t readDataRaw(TwoWire& wire, uint16_t address1, uint16_t address2, uint8_t* dst, size_t len,
                   uint32_t timeout = -1);
uint16_t readDataFromRaw16(uint8_t* rawData, Endianness endian);
uint32_t readDataFromRaw24(uint8_t* rawData, Endianness endian);
}  // namespace Lightus::WireHelper
#endif  // LIGHTUS_SENSORS_WIREHELPER_HPP
