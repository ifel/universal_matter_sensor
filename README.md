# Matter-Based Multi-Sensor (SHT4x, BME280 / BMP280, SCD30, VEML7700) for Temperature, Humidity, Pressure, CO₂, Air Quality, and Illuminance

This project implements a Matter-compatible **Temperature**, **Relative Humidity**, **Pressure**, **Carbon Dioxide (CO₂) Concentration**, **Air Quality**, and **Illuminance (Ambient Light)** sensor on ESP32 microcontrollers (such as ESP32-C6, ESP32-H2, or ESP32) using **ESP-Matter** and the [`esp-idf-lib/sht4x`](https://components.espressif.com/components/esp-idf-lib/sht4x), [`esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280), [`esp-idf-lib/scd30`](https://components.espressif.com/components/esp-idf-lib/scd30), and [`esp-idf-lib/veml7700`](https://components.espressif.com/components/esp-idf-lib/veml7700) components from the ESP Component Registry.

It supports the following I2C sensors (individually or combined on the same I2C bus), listed in priority order for overlapping measurement types:

1. **SHT40 / SHT41 / SHT45 (`SHT4x`)**: Temperature and Relative Humidity
2. **BME280 / BMP280**:
   - **BME280**: Temperature, Pressure, and Relative Humidity
   - **BMP280**: Temperature and Pressure
3. **SCD30**: Carbon Dioxide (CO₂) Concentration (with derived Matter **Air Quality**), Temperature, and Relative Humidity
4. **VEML7700**: Illuminance (Ambient Light in `lx`)

When multiple sensors are connected on the I2C bus, higher-priority sensors supersede lower-priority ones for any overlapping measurement types (`SHT4x` > `BME280 / BMP280` > `SCD30`), while unique measurements (**Pressure** from `BMP280 / BME280`, **CO₂ & Air Quality** from `SCD30`, **Illuminance** from `VEML7700`, or **Relative Humidity** from `SCD30` when paired only with a `BMP280`) are still taken from the sensor that provides them. Additionally, when a pressure sensor (`BMP280 / BME280`) is present alongside an `SCD30`, barometric pressure readings are used to dynamically compensate `SCD30` CO₂ measurements.

---

## Documentation

- [**Home Assistant Commissioning & QR Code Guide**](docs/home_assistant_commissioning.md) — How to pair the device over Matter-over-Wi-Fi or Matter-over-Thread and how to use **ESP Launchpad** or **`esp-matter-mfg-tool`** to simplify flashing and QR code generation.
- [**Matter OTA Firmware Updates Guide**](docs/ota_firmware_updates.md) — How to enable Matter OTA Requestor, build `.ota` upgrade images, and push firmware updates via Home Assistant or Espressif portals.
- [**Sensor Reporting Thresholds & Heartbeat Throttling**](docs/sensor_reporting_thresholds.md) — How deadband filtering (`abs_threshold` + `rel_threshold`) and periodic heartbeat reporting work, with reference datasheet tables for BMP280/BME280, BMP581, TMP119, SHT45, SCD-30, and VEML7700.

---

## Prerequisites

### Hardware

- **Seeed Studio XIAO ESP32C6** (or ESP32-H2 / ESP32 development board)
- Any combination of the following I2C sensor modules:
  - **Sensirion SHT40 / SHT41 / SHT45** Temperature and Relative Humidity Sensor module (e.g., Adafruit SHT40 / SHT41 / SHT45)
  - **Bosch BME280** or **BMP280** Temperature, Humidity (BME280), and Barometric Pressure Sensor module (e.g., Adafruit BME280 / BMP280)
  - **Sensirion SCD30** NDIR CO₂, Temperature, and Relative Humidity Sensor module (e.g., Adafruit SCD-30)
  - **Vishay VEML7700** Ambient Light (Lux) Sensor module (e.g., Adafruit VEML7700)
- Breadboard and connecting wires
- 3.3V power supply

### Software

- **ESP-IDF >= 6.1.0**
- **ESP-Matter**
- **`esp-idf-lib/bmp280`**, **`esp-idf-lib/sht4x`**, **`esp-idf-lib/scd30`**, & **`esp-idf-lib/veml7700`** (automatically fetched via the IDF Component Manager from [`components/sensors_manager/idf_component.yml`](components/sensors_manager/idf_component.yml))

---

## Getting Started

### 1. ESP-IDF & ESP-Matter Environment Setup

Ensure both ESP-IDF and ESP-Matter environments are exported in your terminal session:

```bash
. $HOME/esp/esp-idf/export.sh
. $HOME/esp/esp-matter/export.sh
```

*(Ensure the `ESP_MATTER_PATH` environment variable is set to your `esp-matter` checkout path).*

---

## Wiring (Seeed Studio XIAO ESP32C6 + SHT4x / BME280 / BMP280 / SCD30 / VEML7700)

Connect one or more supported sensor modules (**SHT4x**, **BME280 / BMP280**, **SCD30**, **VEML7700**) in parallel to the **Seeed Studio XIAO ESP32C6** hardware I2C pins:

| Sensor Pin (SHT4x / BME280 / BMP280 / SCD30 / VEML7700) | XIAO ESP32C6 Pin | ESP32-C6 GPIO | Notes |
| --- | --- | --- | --- |
| **VIN** (or **3Vo**) | **3V3** | — | 3.3V power supply |
| **GND** | **GND** | — | Common ground |
| **SDA** / **SDI** | **D4** (SDA) | **GPIO22** | Configurable via `CONFIG_SENSORS_I2C_SDA_GPIO` |
| **SCL** / **SCK** | **D5** (SCL) | **GPIO23** | Configurable via `CONFIG_SENSORS_I2C_SCL_GPIO` |
| **SDO** *(BMP280/BME280 only)* | Unconnected | — | Pulled high on Adafruit module → default I2C address **`0x77`**. Connect to `GND` for `0x76` (`CONFIG_BMP280_I2C_ADDRESS`). **SHT4x** uses fixed I2C address **`0x44`**, **SCD30** uses fixed I2C address **`0x61`**, and **VEML7700** uses fixed I2C address **`0x10`**. |

### Auto-Detection & Sensor Priority

At boot, `SensorsManager` probes registered I2C sensors in priority order (**`Sht4x` → `Bmp280Bme280` → `Scd30` → `Veml7700`**) and assigns each `SensorType` to the first detected sensor that provides it. `MatterManager` then dynamically creates only the Matter endpoints/clusters for the detected measurement types:

| Connected Sensor(s) | Temperature (`0x0402`) | Relative Humidity (`0x0405`) | Pressure (`0x0403`) | CO₂ (`0x040D`) & Air Quality (`0x005B`) | Illuminance (`0x0400`) |
| --- | --- | --- | --- | --- | --- |
| **SHT40 / SHT41 / SHT45** only | **SHT4x** | **SHT4x** | *Not created* | *Not created* | *Not created* |
| **BMP280** only | **BMP280** | *Not created* | **BMP280** | *Not created* | *Not created* |
| **BME280** only | **BME280** | **BME280** | **BME280** | *Not created* | *Not created* |
| **SCD30** only | **SCD30** | **SCD30** | *Not created* | **SCD30** | *Not created* |
| **VEML7700** only | *Not created* | *Not created* | *Not created* | *Not created* | **VEML7700** |
| **SHT4x + BMP280** | **SHT4x** *(supersedes BMP280)* | **SHT4x** | **BMP280** | *Not created* | *Not created* |
| **SHT4x + BME280** | **SHT4x** *(supersedes BME280)* | **SHT4x** *(supersedes BME280)* | **BME280** | *Not created* | *Not created* |
| **SHT4x + SCD30** | **SHT4x** *(supersedes SCD30)* | **SHT4x** *(supersedes SCD30)* | *Not created* | **SCD30** | *Not created* |
| **BMP280 + SCD30** | **BMP280** *(supersedes SCD30)* | **SCD30** | **BMP280** | **SCD30** *(pressure-compensated)* | *Not created* |
| **BME280 + SCD30** | **BME280** *(supersedes SCD30)* | **BME280** *(supersedes SCD30)* | **BME280** | **SCD30** *(pressure-compensated)* | *Not created* |
| **SHT4x + BMP280 / BME280 + SCD30 + VEML7700** | **SHT4x** *(supersedes all)* | **SHT4x** *(supersedes all)* | **BMP280 / BME280** | **SCD30** *(pressure-compensated)* | **VEML7700** |

> **Onboard Status LED**: Driven by the `status_led` component, the onboard yellow User LED on the XIAO ESP32C6 (**`GPIO15`**, active-low, configurable via `CONFIG_STATUS_LED_GPIO`) stays **OFF during boot/init**, turns **solid ON** during normal operation once sensor and Matter initialization succeed, blinks slowly (`1000 ms`) while the Matter commissioning window is open, and blinks rapidly (`200 ms`) when the Matter **Identify** command is triggered from Home Assistant.

---

## Project Structure

- [`main/`](main/app_main.cpp) — Application entry point (`app_main`): initializes NVS, Thread/Wi-Fi, `SensorsManager`, `StatusLed`, and `MatterManager` (restarting after 10 s if sensors or Matter fail to initialize), then periodically polls sensors, passes environmental compensations (such as barometric pressure for `SCD30`), and updates Matter cluster attributes.
- [`components/common_types/`](components/common_types/include/common_types/common_types.h) — Shared [`SensorType`](components/common_types/include/common_types/common_types.h) enum (`TEMPERATURE`, `PRESSURE`, `RELATIVE_HUMIDITY`, `CARBON_DIOXIDE_CONCENTRATION`, `ILLUMINANCE`), [`SensorValueRange`](components/common_types/include/common_types/common_types.h) struct (`°C`, `Pa`, `%RH`, `ppm`, `lx`), and `PerSensorType` containers.
- [`components/sensors_manager/`](components/sensors_manager/include/sensors_manager/sensors_manager.h) — Abstract [`Sensor`](components/sensors_manager/include/sensors_manager/sensor.h) interface, [`Sht4x`](components/sensors_manager/src/sensors/sht4x.h), [`Bmp280Bme280`](components/sensors_manager/src/sensors/bmp280_bme280.h), [`Scd30`](components/sensors_manager/src/sensors/scd30.h), and [`Veml7700`](components/sensors_manager/src/sensors/veml7700.h) driver implementations, and [`SensorsManager`](components/sensors_manager/include/sensors_manager/sensors_manager.h) for probing active sensors in priority order, performing a single hardware read per active chip, and serving readings/ranges by `SensorType`.
- [`components/matter_interface/`](components/matter_interface/include/matter_interface/matter_manager.h) — [`MatterManager`](components/matter_interface/include/matter_interface/matter_manager.h) for creating Matter endpoints, scaling physical units to Matter attributes (including logarithmic lux scaling for `IlluminanceMeasurement` and CO₂-derived `AirQuality`), and updating cluster attributes with deadband + heartbeat filtering.
- [`components/status_led/`](components/status_led/include/status_led/status_led.h) — Thread-safe [`StatusLed`](components/status_led/include/status_led/status_led.h) helper driven by `setReady` (solid on), `setCommissioning` (slow blink) and `setIdentifying` (fast blink). It has no Matter dependency; `app_main` connects it to `MatterManager` listeners.

---

## Build and Flash

The project provides [`CMakePresets.json`](CMakePresets.json) to build separate **development**, **production Thread**, and **production Wi-Fi** firmwares into isolated build directories (`build/default`, `build/prod`, and `build/prod_wifi`), with distinct Matter Product IDs (`0x8001` for Thread vs `0x8000` for Wi-Fi) so OTA updates never cross-flash the wrong radio variant:

| CMake Preset (`--preset`) | Build Directory | Radio Transport | Configuration Files | Matter Product ID (`CONFIG_DEVICE_PRODUCT_ID`) | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`default`** | `build/default` | **Matter over Wi-Fi** | [`sdkconfig.defaults`](sdkconfig.defaults) | `0x8000` | Development configuration |
| **`prod`** | `build/prod` | **Matter over Thread** | [`sdkconfig.defaults`](sdkconfig.defaults) + [`sdkconfig.defaults.prod_common`](sdkconfig.defaults.prod_common) + [`sdkconfig.defaults.prod`](sdkconfig.defaults.prod) | `0x8001` | Production Thread configuration (`-O2` performance optimization, OpenThread enabled, Wi-Fi disabled) |
| **`prod_wifi`** | `build/prod_wifi` | **Matter over Wi-Fi** | [`sdkconfig.defaults`](sdkconfig.defaults) + [`sdkconfig.defaults.prod_common`](sdkconfig.defaults.prod_common) + [`sdkconfig.defaults.prod_wifi`](sdkconfig.defaults.prod_wifi) | `0x8000` | Production Wi-Fi configuration (`-O2` performance optimization, Wi-Fi enabled, Thread disabled) |

### Step 1. Set Target Chip

Specify your desired preset (`default`, `prod`, or `prod_wifi`) with `idf.py --preset`:

```bash
idf.py --preset prod set-target esp32c6        # Production Matter-over-Thread
# or:
idf.py --preset prod_wifi set-target esp32c6   # Production Matter-over-Wi-Fi
# or:
idf.py --preset default set-target esp32c6     # Development (Matter-over-Wi-Fi)
```

### Step 2. Configure Settings (Optional)

Project settings are organized by component in `idf.py menuconfig`:

- **`Application Configuration`** ([`main/Kconfig.projbuild`](main/Kconfig.projbuild)): Sensor polling interval (`CONFIG_SENSOR_READ_DELAY_MS`, default `10000 ms`).
- **`Sensors Configuration`** ([`components/sensors_manager/Kconfig.projbuild`](components/sensors_manager/Kconfig.projbuild)): Shared I2C port (`CONFIG_SENSORS_I2C_PORT`, default `0`), SDA/SCL GPIOs (`CONFIG_SENSORS_I2C_SDA_GPIO` = `22`, `CONFIG_SENSORS_I2C_SCL_GPIO` = `23`), and BMP280/BME280 I2C address (`0x77` default for Adafruit modules, or `0x76`).
- **`Matter Manager Configuration`** ([`components/matter_interface/Kconfig.projbuild`](components/matter_interface/Kconfig.projbuild)): Maximum reporting interval (`CONFIG_MATTER_REPORT_MAX_INTERVAL_S`, default `180 s`).
- **`Status LED Configuration`** ([`components/status_led/Kconfig.projbuild`](components/status_led/Kconfig.projbuild)): Status LED GPIO (`15` default, `-1` to disable) and active-low polarity (`CONFIG_STATUS_LED_ACTIVE_LOW`).

```bash
idf.py --preset prod menuconfig
```

### Step 3. Build, Flash, and Monitor

- **Production Matter over Thread (`prod` → `build/prod/`)**:

  ```bash
  idf.py --preset prod build
  idf.py --preset prod -p <PORT> flash monitor
  ```

- **Production Matter over Wi-Fi (`prod_wifi` → `build/prod_wifi/`)**:

  ```bash
  idf.py --preset prod_wifi build
  idf.py --preset prod_wifi -p <PORT> flash monitor
  ```

- **Development (`default` → `build/default/`)**:

  ```bash
  idf.py --preset default build
  idf.py --preset default -p <PORT> flash monitor
  ```

Press `CTRL+]` to exit the serial monitor.

Once running, follow the [**Home Assistant Commissioning Guide**](docs/home_assistant_commissioning.md) to add the sensor to Home Assistant.

---

## Acknowledgements

This project is based on the original [matter_bmp280](https://github.com/albert-gee/matter_bmp280) project by **Albert Gee** ([`@albert-gee`](https://github.com/albert-gee)). It has been extended to support multi-sensor auto-detection (`SHT40`/`SHT41`/`SHT45`, `BME280`/`BMP280`, `SCD30`, and `VEML7700`) using the `esp-idf-lib/sht4x`, `esp-idf-lib/bmp280`, `esp-idf-lib/scd30`, and `esp-idf-lib/veml7700` managed components.

---

## License

This project is licensed under the MIT License. See the [`LICENSE`](LICENSE) file for details.

---

## References

- [Sensirion SHT4x (SHT40 / SHT41 / SHT45) Datasheet](https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf)
- [Bosch BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Bosch BMP280 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)
- [Sensirion SCD30 Datasheet](https://sensirion.com/media/documents/4EAF6AF8/61652C3C/Sensirion_CO2_Sensors_SCD30_Datasheet.pdf)
- [Vishay VEML7700 Datasheet](https://www.vishay.com/docs/84286/veml7700.pdf)
- [ESP Component Registry — `esp-idf-lib/sht4x`](https://components.espressif.com/components/esp-idf-lib/sht4x)
- [ESP Component Registry — `esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280)
- [ESP Component Registry — `esp-idf-lib/scd30`](https://components.espressif.com/components/esp-idf-lib/scd30)
- [ESP Component Registry — `esp-idf-lib/veml7700`](https://components.espressif.com/components/esp-idf-lib/veml7700)
- [Espressif Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [Matter Protocol Specification](https://csa-iot.org/all-solutions/matter/)
