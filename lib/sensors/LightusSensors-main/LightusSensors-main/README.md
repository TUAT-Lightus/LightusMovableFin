# Lightus_Sensors
Arduino用センサーライブラリ  
BMX055,BME280対応  

## 使用方法  
example/内にサンプルコードが入っています  
### セットアップ
1. I²Cのセットアップ(`Wire.begin();`etc.)
1. `bme280.begin(Wire);`、`bmx055.begin(Wire);`する
### 計測範囲指定
BMX055からデータを取得する際、デフォルトでは加速度は±2G、角加速度は±500°/sの範囲内の値で返ってくる  
これを変えるには、`bmx055.begin()`関数に`Lightus::Sensors::BME280::AccelRange`列挙型、`Lightus::Sensors::BME280::GyroRange`列挙型の引数を入れる  
例はexamples/MaximumRange.inoを参照  
### データ取得
BMEからのデータは、`bme280.read()`すると`Lightus::Sensors::BME280::DataType`型で返ってくる。  
BMXからのデータは、`bmx055.read()`すると、`Lightus::Sensors::BMX055::DataType`型で返ってくる。  
BMXに関しては、`bmx055.readAccel()`,`bmx055.readGyro()`,`bmx055.readMag()`すると各データが`Lightus::Vec3<float>`型で得られる。  
BMEに関しては、データの補正の都合上個別に得る関数はない。  
変換前の16bitの加速度データ(+α)は、`bmx055.readAccel16()`すると、`Lightus::Vec3<int16_t>`型で返ってくる。  
変換前の16bitの角速度データ(+α)は、`bmx055.readGyro16()`すると、`Lightus::Vec3<int16_t>`型で返ってくる。  
### データ解析
以下では、BMEからのデータを`bmeData`、BMXからのデータを`bmxData`に入れたとして説明する。  
#### BMEのデータ
* 以下のデータの型は全て`float`
* 気温データ[°C]は`bmeData.temperature`  
* 気圧データ[hPa]は`bmeData.pressure`
* 湿度データ[%]は`bmeData.humidity`
#### BMXのデータ
* データの型は全て`Lightus::Vec3<float>`
* 加速度[g]データは`bmxData.accel`
* 角速度[°/s]データは`bmxData.gyro`
* 地磁気データ[μT]は`bmxData.mag`
#### Vec3
`Vec3<T>`は3次元ベクトルを表す。`T`は各軸の値の型。  
以下では、`Vec3<float> vec`変数があるとして説明する。
* x軸のデータは`vec.x`
* y軸のデータは`vec.y`
* z軸のデータは`vec.z`  

例えば、z軸加速度を得るには、`bmxData.accel.z`にアクセスすればよい。  
#### 16bitからfloatへの変換
16bitのデータからfloatのデータに変換するには、`Lightus::Sensors::BMX055::convertAccelFrom16(accel16,accelRange)`関数、`Lightus::Sensors::BMX055::convertGyroFrom16(gyro16,gyroRange)`関数を使う。  
両関数は静的関数なのでbmx055.begin()しなくとも使える。

### その他  
いちいち`Lightus::Sensors::BMX055`などと書いているとコードが長くなるので、`using Lightus::Sensors::BMX055;`とすると以降では`BMX055`と書けばよくなる。

## 参考文献  
[BMX055データシート](https://akizukidenshi.com/download/ds/bosch/BST-BMX055-DS000.pdf)  
[BME280データシート](https://akizukidenshi.com/download/ds/bosch/BST-BME280_DS001-10.pdf)  
[BMM150(BMX055の地磁気センサー)用Bosch社公式ライブラリ](https://github.com/boschsensortec/BMM150-Sensor-API/tree/master)  
