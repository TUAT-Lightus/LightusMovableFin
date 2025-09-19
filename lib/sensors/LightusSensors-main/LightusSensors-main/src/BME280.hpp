#ifndef LIGHTUS_SENSORS_BME280_HPP
#define LIGHTUS_SENSORS_BME280_HPP
#include <ArduinoJson.h>
#include <Wire.h>

#include <array>

#include "EnvironmentSensorData.hpp"
namespace Lightus::Sensors {
class BME280 {
   public:
    enum class TemperatureOversampling {
        SKIP = 0b000,
        TM1  = 0b001,
        TM2  = 0b010,  // Temperature oversampling x 2
        TM4  = 0b011,
        TM8  = 0b100,
        TM16 = 0b101,
    };
    enum class PressureOversampling {
        SKIP = 0b000,
        TM1  = 0b001,
        TM2  = 0b010,
        TM4  = 0b011,
        TM8  = 0b100,
        TM16 = 0b101,  // Pressure oversampling    x 16
    };
    enum class HumidityOversampling {
        SKIP = 0b000,
        TM1  = 0b001,  // Humidity oversampling    x 1
        TM2  = 0b010,
        TM4  = 0b011,
        TM8  = 0b100,
        TM16 = 0b101,
    };
    enum class Mode {
        SLEEP  = 0b00,
        FORCED = 0b01,  // 0b10も同じ
        NORMAL = 0b11,  // Normal mode
    };
    enum class Tstandby {
        ms0_5  = 0b000,  // Tstandby 0.5ms
        ms62_5 = 0b001,
        ms125  = 0b010,
        ms250  = 0b011,
        ms500  = 0b100,
        ms1000 = 0b101,
        ms10   = 0b110,
        ms20   = 0b111,
    };
    enum class Filter {
        OFF   = 0b000,
        IIR2  = 0b001,
        IIR4  = 0b010,
        IIR8  = 0b011,
        IIR16 = 0b100,  // IIR Filter x16
    };
    enum class SPI3WireEnable {
        DISABLE = 0,  // 3-wire SPI Disable
        ENABLE  = 1,
    };
    /// @brief BME280を起動する
    /// @param osrs_t 温度計のオーバーサンプリング量
    /// @param osrs_p 気圧計のオーバーサンプリング量
    /// @param osrs_h 湿度計のオーバーサンプリング量
    /// @param mode モード
    /// @param t_sb スタンバイ時間
    /// @param filter IIRフィルター設定
    /// @param spi3w_en 3-wire SPI 有効/無効
    /// @warning この関数を呼ぶ前にWire.begin()すること！！！
    void begin(TwoWire& wire, TemperatureOversampling osrs_t = TemperatureOversampling::TM2,
               PressureOversampling osrs_p = PressureOversampling::TM16,
               HumidityOversampling osrs_h = HumidityOversampling::TM1, Mode mode = Mode::NORMAL,
               Tstandby t_sb = Tstandby::ms0_5, Filter filter = Filter::IIR16,
               SPI3WireEnable spi3w_en = SPI3WireEnable::DISABLE);

    using DataType = EnvironmentSensorData<float>;
    /// @brief BME280からデータを読み出す
    /// @return 温度、気圧、湿度のデータ
    /// @warning 温度、湿度については未実装なので、気圧しか正しい値が返ってこない
    /// @todo 温度、湿度についても実装する
    DataType read();

    using RawDataType = std::array<uint8_t, 8>;
    RawDataType readRaw();
    String readCompensationParametersJSON();
    void dumpCompensationParametersToJSON(JsonObject obj);

    void setAddress(uint8_t addr);
    uint8_t address() const noexcept;

   private:
    TwoWire* wire_;
    uint8_t BME280_ADDRESS_ = 0x76;
    void readTrim_();

    /// @brief 生のバイト列から気圧の値を読み取る
    /// @param rawData 生の気圧データ
    /// @warning ~rawData[0]~が気圧データの最初のバイトになるようにポインタを進めておくこと
    /// @return 補正等もされた気圧の値
    float readPressure_(uint8_t* rawData);

    /// @brief 生のバイト列から気温の値を読み取る
    /// @param rawData 生の気温データ
    /// @warning ~rawData[0]~が気温データの最初のバイトになるようにポインタを進めておくこと
    /// @return 補正等もされた気温の値
    float readTemperature_(uint8_t* rawData);

    /// @brief 生のバイト列から気圧の値を読み取る
    /// @param rawData 生の気圧データ
    /// @warning ~rawData[0]~が気圧データの最初のバイトになるようにポインタを進めておくこと
    /// @return 補正等もされた気圧の値
    float readHumidity_(uint8_t* rawData);

    /// @brief 気圧の生データを、補正データを用いて補正する
    /// @param adc_P
    /// @return 上位24bitが整数部分、下位8ビットが小数点以下の気圧データ、つまり256倍された値。単位はPa
    /// @note BOSCH社が公開しているBME280のデータシートの4.2.3 Compensation formulas (p.23) 参照
    uint32_t computePres64_(int32_t adc_P);

    /// @brief 気温の生データを、補正データを用いて補正する
    /// @param adc_T
    /// @return 100倍されたセ氏温度(e.g. 12.34°C → 1234)
    /// @note ~t_fine_~を更新するので、~t_fine_~が必要な処理より前に呼ぶこと。
    /// @note BOSCH社が公開しているBME280のデータシートの4.2.3 Compensation formulas (p.23) 参照
    uint32_t computeTemp64_(int32_t adc_T);

    /// @brief 湿度の生データを補正データを用いて補正する
    /// @param adc_H
    /// @return 上位22bitが整数部分、下位10bitが小数点以下の気圧データ、つまり1024倍された値。単位は%RH
    uint32_t computeHum64_(int32_t adc_H);

    int8_t dig_H6_;
    uint8_t dig_H1_, dig_H3_;
    int16_t dig_T2_, dig_T3_, dig_P2_, dig_P3_, dig_P4_, dig_P5_, dig_P6_, dig_P7_, dig_P8_, dig_P9_;
    int16_t dig_H2_, dig_H4_, dig_H5_;
    uint16_t dig_T1_, dig_P1_;

    int32_t t_fine_;
};
}  // namespace Lightus::Sensors
#if !defined(LIGHTUS_SENSORS_NO_BME280_INSTANCE) && !defined(LIGHTUS_SENSORS_NO_GLOBAL_INSTANCES)
extern Lightus::Sensors::BME280 bme280;
#endif

#endif  // LIGHTUS_SENSORS_BME280_HPP
