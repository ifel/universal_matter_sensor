# Matter-Based Multi-Sensor (TMP119, SHT4x, BMP581, BME280 / BMP280, SCD30, ENS160, VEML7700, TSL2591, VL53L1X, MAX17048, LD2410S) for Temperature, Humidity, Pressure, CO₂, TVOC, Air Quality, Illuminance, Battery, and Occupancy

This project implements a Matter-compatible **Temperature**, **Relative Humidity**, **Pressure**, **Carbon Dioxide (CO₂) Concentration**, **Total Volatile Organic Compounds (TVOC) Concentration**, **Air Quality**, **Illuminance (Ambient Light)**, **Remaining Level** (e.g. a water softener's salt block, via a ToF distance sensor), **Battery** (percentage, voltage, charge state via a fuel gauge), and **Occupancy** (mmWave presence radar) sensor on ESP32 microcontrollers (such as ESP32-C6, ESP32-H2, or ESP32) using **ESP-Matter** and the [`esp-idf-lib/sht4x`](https://components.espressif.com/components/esp-idf-lib/sht4x), [`esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280), [`esp-idf-lib/scd30`](https://components.espressif.com/components/esp-idf-lib/scd30), and [`esp-idf-lib/veml7700`](https://components.espressif.com/components/esp-idf-lib/veml7700), [`esp-idf-lib/tsl2591`](https://components.espressif.com/components/esp-idf-lib/tsl2591), and [`esp-idf-lib/max1704x`](https://components.espressif.com/components/esp-idf-lib/max1704x) components (plus [`grrtzm/vl53l1x_library`](https://components.espressif.com/components/grrtzm/vl53l1x_library), ST's VL53L1X ULP driver) from the ESP Component Registry, plus a local UART driver, [`ld2410s`](components/ld2410s) (HLK-LD2410S presence radar), and three local drivers on the esp-idf-lib `i2cdev` bus layer: [`tmp117`](components/tmp117) (TMP117 / TMP119), [`ens160`](components/ens160) (a port of [`k0i05/esp_ens160`](https://components.espressif.com/components/k0i05/esp_ens160)) and [`bmp5`](components/bmp5) (Bosch's [BMP5 SensorAPI](https://github.com/boschsensortec/BMP5_SensorAPI) with `i2cdev` glue).

It supports the following I2C sensors (individually or combined on the same I2C bus), listed in priority order for overlapping measurement types. Sensors are ordered by the accuracy of their main measurement, most accurate first. For example, the **SCD30**'s NDIR CO₂ is a true measurement and far more accurate than the **ENS160**'s estimated eCO₂, so the SCD30 is probed first:

1. **TMP119 / TMP117**: Temperature (±0.08 °C max from 0 to 45 °C on the TMP119; 8× averaged one-shot per read, low self-heating)
2. **SHT40 / SHT41 / SHT45 (`SHT4x`)**: Temperature and Relative Humidity
3. **BMP581 / BMP585**: Temperature and Pressure (much more precise pressure than the BMP280)
4. **BME280 / BMP280**:
   - **BME280**: Temperature, Pressure, and Relative Humidity
   - **BMP280**: Temperature and Pressure
5. **SCD30**: Carbon Dioxide (CO₂) Concentration (with derived Matter **Air Quality**), Temperature, and Relative Humidity
6. **ENS160**: Equivalent CO₂ (eCO₂), TVOC (`ppb`), and Air Quality (from the sensor's UBA index)
7. **VEML7700**: Illuminance (Ambient Light in `lx`; auto-ranged gain, with Vishay's non-linearity correction above `1000 lx` at gain 1/8 and 1/4)
8. **TSL2591**: Illuminance (Ambient Light in `lx`, IR-compensated, better in low light; up to ~88,000 lx)
9. **VL53L1X**: Remaining Level (`%`), computed on the device from a time-of-flight distance (see [Remaining Level](#remaining-level-vl53l1x))
10. **MAX17048 / MAX17049**: Battery percentage, voltage, and charge state of the device's own single-cell Li-ion / LiPo battery (see [Battery](#battery-max17048))
11. **HLK-LD2410S** *(UART, not I2C)*: Occupancy from a 24 GHz mmWave presence radar, pushed to Matter as soon as it changes (see [Occupancy](#occupancy-hlk-ld2410s))

When multiple sensors are connected on the I2C bus, higher-priority sensors supersede lower-priority ones for any overlapping measurement types (`TMP119` > `SHT4x` > `BMP581` > `BME280 / BMP280` > `SCD30` > `ENS160`, and `VEML7700` > `TSL2591`), while unique measurements (**Pressure** from `BMP581` or `BMP280 / BME280`, **Relative Humidity** from a `BME280` paired with a `BMP581`, **CO₂** from `SCD30` (or `ENS160` without an `SCD30`), **TVOC** from `ENS160`, **Illuminance** from `VEML7700` (or `TSL2591` without a `VEML7700`), or **Relative Humidity** from `SCD30` when paired only with a `BMP280`) are still taken from the sensor that provides them. Additionally, when a pressure sensor (`BMP581` or `BMP280 / BME280`) is present alongside an `SCD30`, barometric pressure readings are used to dynamically compensate `SCD30` CO₂ measurements, and temperature/humidity readings compensate `ENS160` measurements. **Air Quality** is not owned by a single sensor: every active sensor that provides CO₂ or TVOC and reports an air quality level contributes, and the **worst** level is published (`SCD30` derives it from CO₂, `ENS160` reports its UBA index).

---

## Documentation

- [**Home Assistant Commissioning & QR Code Guide**](docs/home_assistant_commissioning.md) — How to pair the device over Matter-over-Wi-Fi or Matter-over-Thread and how to use **ESP Launchpad** or **`esp-matter-mfg-tool`** to simplify flashing and QR code generation.
- [**Matter OTA Firmware Updates Guide**](docs/ota_firmware_updates.md) — How to enable Matter OTA Requestor, build `.ota` upgrade images, and push firmware updates via Home Assistant or Espressif portals.
- [**Sensor Reporting Thresholds & Heartbeat Throttling**](docs/sensor_reporting_thresholds.md) — How deadband filtering (`abs_threshold` + `rel_threshold`) and periodic heartbeat reporting work, with reference datasheet tables for BMP280/BME280, BMP581, TMP119, SHT45, SCD-30, ENS160, VEML7700, and TSL2591.

---

## Prerequisites

### Hardware

- **Seeed Studio XIAO ESP32C6** (or ESP32-H2 / ESP32 development board)
- Any combination of the following I2C sensor modules:
  - **TI TMP119** (or **TMP117**) High-Accuracy Temperature Sensor module (e.g., Adafruit TMP119 / TMP117)
  - **Sensirion SHT40 / SHT41 / SHT45** Temperature and Relative Humidity Sensor module (e.g., Adafruit SHT40 / SHT41 / SHT45)
  - **Bosch BMP581** (or **BMP585**) high-precision Barometric Pressure and Temperature Sensor module (e.g., Adafruit / SparkFun BMP581)
  - **Bosch BME280** or **BMP280** Temperature, Humidity (BME280), and Barometric Pressure Sensor module (e.g., Adafruit BME280 / BMP280)
  - **Sensirion SCD30** NDIR CO₂, Temperature, and Relative Humidity Sensor module (e.g., Adafruit SCD-30)
  - **ScioSense ENS160** Digital Metal-Oxide Multi-Gas (eCO₂ / TVOC / AQI) Sensor module (e.g., SparkFun / Adafruit ENS160)
  - **Vishay VEML7700** Ambient Light (Lux) Sensor module (e.g., Adafruit VEML7700)
  - **ams TSL2591** High Dynamic Range Light (Lux) Sensor module (e.g., Adafruit TSL2591)
  - **Hi-Link HLK-LD2410S** low-power 24 GHz mmWave Human Presence Sensor (UART; *not* the LD2410 / LD2410B / LD2410C)
  - **Analog Devices MAX17048 / MAX17049** LiPo Fuel Gauge module (e.g., Adafruit / SparkFun MAX17048), wired to the device's battery
  - **ST VL53L1X** Time-of-Flight Distance Sensor module (e.g., Pololu / Adafruit / SparkFun VL53L1X)
- Breadboard and connecting wires
- 3.3V power supply

### Software

- **ESP-IDF >= 6.1.0**
- **ESP-Matter**
- **`esp-idf-lib/bmp280`**, **`esp-idf-lib/sht4x`**, **`esp-idf-lib/scd30`**, **`esp-idf-lib/veml7700`**, **`esp-idf-lib/tsl2591`**, & **`grrtzm/vl53l1x_library`** (automatically fetched via the IDF Component Manager from [`components/sensors_manager/idf_component.yml`](components/sensors_manager/idf_component.yml))

---

## Getting Started

### 0. Clone with Submodules

The Bosch BMP5 SensorAPI is a git submodule (`components/bmp5/third_party`):

```bash
git clone --recursive <repo-url>
# or, in an existing clone:
git submodule update --init
```

### 1. ESP-IDF & ESP-Matter Environment Setup

Ensure both ESP-IDF and ESP-Matter environments are exported in your terminal session:

```bash
. $HOME/esp/esp-idf/export.sh
. $HOME/esp/esp-matter/export.sh
```

*(Ensure the `ESP_MATTER_PATH` environment variable is set to your `esp-matter` checkout path).*

---

## Wiring (Seeed Studio XIAO ESP32C6 + TMP119 / SHT4x / BMP581 / BME280 / BMP280 / SCD30 / ENS160 / VEML7700 / TSL2591 / VL53L1X)

Connect one or more supported sensor modules (**SHT4x**, **BMP581**, **BME280 / BMP280**, **SCD30**, **ENS160**, **VEML7700**, **TSL2591**, **VL53L1X**) in parallel to the **Seeed Studio XIAO ESP32C6** hardware I2C pins:

| Sensor Pin (TMP119 / SHT4x / BMP581 / BME280 / BMP280 / SCD30 / ENS160 / VEML7700 / TSL2591 / VL53L1X) | XIAO ESP32C6 Pin | ESP32-C6 GPIO | Notes |
| --- | --- | --- | --- |
| **VIN** (or **3Vo**) | **3V3** | — | 3.3V power supply |
| **GND** | **GND** | — | Common ground |
| **SDA** / **SDI** | **D4** (SDA) | **GPIO22** | Configurable via `CONFIG_SENSORS_I2C_SDA_GPIO` |
| **SCL** / **SCK** | **D5** (SCL) | **GPIO23** | Configurable via `CONFIG_SENSORS_I2C_SCL_GPIO` |
| **SDO** *(BMP280/BME280 only)* | Unconnected | — | Pulled high on Adafruit module → I2C address **`0x77`**; connected to `GND` → `0x76`. **BMP581** uses **`0x47`** (SDO high on Adafruit / SparkFun modules) or `0x46` (SDO to `GND`). Both addresses are probed automatically for these and ENS160. **TMP119 / TMP117** uses **`0x48`** (ADD0 to `GND`, the usual module default), `0x49` (V+), `0x4A` (SDA) or `0x4B` (SCL); all four are probed. **SHT4x** uses fixed I2C address **`0x44`**, **SCD30** uses fixed I2C address **`0x61`**, **ENS160** uses **`0x53`** (ADDR high) or `0x52` (ADDR to `GND`), and **VEML7700** uses fixed I2C address **`0x10`**, and **TSL2591** uses fixed I2C address **`0x29`**, **VL53L1X** defaults to **`0x29`** too (so it can't share a bus with a TSL2591), and **MAX17048** uses fixed I2C address **`0x36`**. |
| **LD2410S** `RX` / `OT1` (TX) *(UART)* | **D6** (TX) / **D7** (RX) | **GPIO16** / **GPIO17** | ESP TX → module RX, module TX (`OT1`) → ESP RX; `UART1`, 115200 8N1 (`CONFIG_LD2410S_UART_PORT`, `CONFIG_LD2410S_TX_GPIO`, `CONFIG_LD2410S_RX_GPIO`). Power it from **3V3**. GPIO16/17 are also UART0's default pins, so the console runs on the USB-C port (USB Serial/JTAG, set in [`sdkconfig.defaults.esp32c6`](sdkconfig.defaults.esp32c6)). |

### Auto-Detection & Sensor Priority

At boot, `SensorsManager` probes registered I2C sensors in priority order (**`Tmp117` → `Sht4x` → `Bmp581` → `Bmp280Bme280` → `Scd30` → `Ens160` → `Veml7700` → `Vl53l1x` → `Tsl2591` → `Max17048` → `Ld2410s`**) and assigns each `SensorType` to the first detected sensor that provides it. `MatterManager` then dynamically creates only the Matter endpoints/clusters for the detected measurement types. CO₂, TVOC, and Air Quality share a single **Air Quality Sensor** endpoint; each cluster on it is populated only when a sensor provides that type. Air Quality is the worst level reported by the sensors that provide CO₂ or TVOC:

| Connected Sensor(s) | Temperature (`0x0402`) | Relative Humidity (`0x0405`) | Pressure (`0x0403`) | CO₂ (`0x040D`) | TVOC (`0x042E`) | Air Quality (`0x005B`) | Illuminance (`0x0400`) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| **TMP119 / TMP117** only | **TMP119** | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* |
| **TMP119 + SHT4x** | **TMP119** *(supersedes SHT4x)* | **SHT4x** | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* |
| **SHT40 / SHT41 / SHT45** only | **SHT4x** | **SHT4x** | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* |
| **BMP280** only | **BMP280** | *Not created* | **BMP280** | *Not created* | *Not created* | *Not created* | *Not created* |
| **BME280** only | **BME280** | **BME280** | **BME280** | *Not created* | *Not created* | *Not created* | *Not created* |
| **SCD30** only | **SCD30** | **SCD30** | *Not created* | **SCD30** | *Not created* | **SCD30** *(CO₂-derived)* | *Not created* |
| **ENS160** only | *Not created* | *Not created* | *Not created* | **ENS160** *(eCO₂)* | **ENS160** | **ENS160** *(UBA index)* | *Not created* |
| **VEML7700** only | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | **VEML7700** |
| **TSL2591** only | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | **TSL2591** |
| **VEML7700 + TSL2591** | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | *Not created* | **VEML7700** *(supersedes TSL2591)* |
| **BMP581** only | **BMP581** | *Not created* | **BMP581** | *Not created* | *Not created* | *Not created* | *Not created* |
| **SHT4x + BMP280** | **SHT4x** *(supersedes BMP280)* | **SHT4x** | **BMP280** | *Not created* | *Not created* | *Not created* | *Not created* |
| **SHT4x + BMP581** | **SHT4x** *(supersedes BMP581)* | **SHT4x** | **BMP581** | *Not created* | *Not created* | *Not created* | *Not created* |
| **BMP581 + BME280** | **BMP581** *(supersedes BME280)* | **BME280** | **BMP581** *(supersedes BME280)* | *Not created* | *Not created* | *Not created* | *Not created* |
| **SHT4x + BME280** | **SHT4x** *(supersedes BME280)* | **SHT4x** *(supersedes BME280)* | **BME280** | *Not created* | *Not created* | *Not created* | *Not created* |
| **SHT4x + SCD30** | **SHT4x** *(supersedes SCD30)* | **SHT4x** *(supersedes SCD30)* | *Not created* | **SCD30** | *Not created* | **SCD30** | *Not created* |
| **SHT4x + ENS160** | **SHT4x** | **SHT4x** | *Not created* | **ENS160** *(T/RH-compensated)* | **ENS160** *(T/RH-compensated)* | **ENS160** | *Not created* |
| **BMP280 + SCD30** | **BMP280** *(supersedes SCD30)* | **SCD30** | **BMP280** | **SCD30** *(pressure-compensated)* | *Not created* | **SCD30** | *Not created* |
| **BME280 + SCD30** | **BME280** *(supersedes SCD30)* | **BME280** *(supersedes SCD30)* | **BME280** | **SCD30** *(pressure-compensated)* | *Not created* | **SCD30** | *Not created* |
| **SCD30 + ENS160** | **SCD30** | **SCD30** | *Not created* | **SCD30** *(supersedes ENS160)* | **ENS160** | Worst of **SCD30** & **ENS160** | *Not created* |
| **SHT4x + BMP581 + SCD30 + ENS160 + VEML7700** | **SHT4x** *(supersedes all)* | **SHT4x** *(supersedes all)* | **BMP581** | **SCD30** *(pressure-compensated)* | **ENS160** | Worst of **SCD30** & **ENS160** | **VEML7700** |

**Recovery**: a sensor whose read fails 3 times in a row (not counting "no result yet", such as warm-up or no VL53L1X target) is re-initialized after an I2C bus reset; its values are null until it recovers, and a failed re-init is retried after 3 more failures. The set of sensors and endpoints is fixed at boot.

### Matter Attributes Beyond the Measured Value

- **Tolerance**: Temperature, Relative Humidity, Pressure and Illuminance publish the owning sensor's datasheet accuracy as their `Tolerance` attribute (Pressure also as `ScaledTolerance`). SHT4x uses the SHT40/41 figures, since the variant is not read; the SCD30's CO₂ accuracy (`±(30 ppm + 3%)`) is not a constant, so CO₂ has none.
- **Pressure**: the *Extended* feature adds `ScaledValue` with `Scale = -1` (`10 Pa` = `0.1 hPa` steps) next to `MeasuredValue` (`1 hPa`). It is reported with the same deadband (`50 Pa`).
- **CO₂**: the *Peak Measurement* and *Average Measurement* features publish the peak and average of the last hour (`PeakMeasuredValueWindow` / `AverageMeasuredValueWindow` = `3600 s`). The window slides in 5-minute steps, and both are refreshed every 5 minutes.
- **Power Source**: without a MAX17048, the root endpoint carries a *Wired* (DC) Power Source powering every endpoint; with one, see [Battery](#battery-max17048).

### Remaining Level (VL53L1X)

The **VL53L1X** measures the distance to a consumable (e.g. the top of a water softener's salt block) and the device converts it to **% remaining**: `CONFIG_VL53L1X_FULL_DISTANCE_MM` (default `30`) is 100%, `FULL_DISTANCE + CONFIG_VL53L1X_LEVEL_HEIGHT_MM` (default `30 + 290 = 320 mm`) is 0%, linear in between and clamped. Each reading is the median of 5 valid samples.

- **Matter**: Matter has no distance or generic level cluster that Home Assistant supports, so the level is published as **HEPA Filter Monitoring `Condition`** (`0x0071`, whole percent, degradation direction *down*) on its own endpoint with the **Air Purifier** device type (without Fan Control, so no dummy fan appears). This is a deliberate spec deviation: Air Purifier requires the Fan Control cluster. Home Assistant accepts the endpoint; other controllers (Apple Home, Google Home, SmartThings) may hide it or the whole device, and the device could not pass Matter certification this way. Home Assistant shows it as *"HEPA filter condition"* — rename the entity (e.g. "Salt remaining"). Until the first reading it reports `100 %`; an unavailable reading keeps the last value (`Condition` is not nullable).
- **Re-teaching "full"**: the endpoint supports `ResetCondition`, which Home Assistant shows as a *"Reset filter condition"* button. After installing a new block, press it: the condition jumps to `100 %` and the next valid distance becomes the new full point, saved in NVS (namespace `vl53l1x`) and used instead of `CONFIG_VL53L1X_FULL_DISTANCE_MM` from then on. Press it only with a full block. The level height stays a build-time setting. A Matter factory reset does not clear the taught value; erasing flash does.
- **Field of view**: the sensor uses its narrowest ROI (4×4 SPADs, ~15° instead of 27°). The footprint diameter is ≈ `2 · d · tan(7.5°)`, i.e. ~84 mm at 320 mm — just inside an 85 × 85 mm block, so **mount the sensor centered above the block**.
- **Minimum range**: the VL53L1X is specified from **40 mm**; a full block closer than that reads less reliably, so mount the sensor a bit higher if possible.
- **I2C address `0x29`** is shared with the **TSL2591**: the two cannot be on the same bus. Both drivers check the chip ID, so the one that is present is detected correctly.

### Battery (MAX17048)

The **MAX17048** (or MAX17049) fuel gauge reports the device's own battery, published on its own endpoint with the **Power Source** device type and cluster (`0x002F`, *Battery* + *Rechargeable* features). Its `EndpointList` names every other endpoint, i.e. the battery powers the whole device.

- **Battery %** → `BatPercentRemaining` (half-percent steps; reported on a `1 %` change). Home Assistant shows it as the device's *Battery* entity.
- **Voltage** → `BatVoltage` (`mV`; reported on a `10 mV` change), shown as *Battery voltage*.
- **Charge state** → `BatChargeState`: *charging* while the gauge's charge rate is above `+1 %/h`, *not charging* while it is below `-1 %/h` (discharging) or the charge is below `99 %`, *full* otherwise. A device drawing less than `1 %/h` on battery can still show *full* (the gauge cannot see the charger). Shown as *Battery charge state*.
- **`BatChargeLevel`** (mandatory) is derived from the percentage: *Warning* at `≤ CONFIG_MATTER_BATTERY_WARNING_PERCENT` (default `20`), *Critical* at `≤ CONFIG_MATTER_BATTERY_CRITICAL_PERCENT` (default `10`).
- The gauge's percentage comes from its ModelGauge algorithm with the generic battery model (no custom calibration); it settles after a few charge/discharge cycles. Its `RCOMP` is temperature-compensated from the device's temperature reading (standing in for the cell's).

### Occupancy (HLK-LD2410S)

The **HLK-LD2410S** is detected over UART (it must answer the *enable configuration* command), switched to its minimal report format at `4 Hz`, and its own "no one" delay is set to the minimum (`10 s`). A reader task parses the reports and publishes occupancy changes **immediately**, without waiting for the sensor read interval.

- **Matter**: **Occupancy Sensor** endpoint with the **Occupancy Sensing** cluster (`0x0406`, *Radar* feature). Home Assistant shows an *Occupancy* binary sensor.
- **Hold time**: `OccupancySensing.HoldTime` (`1..300 s`, default `30 s`) delays *unoccupied* in firmware and appears in Home Assistant as a writable *Hold time* setting; the Matter stack persists it across reboots. The total off-delay is the radar's `10 s` plus the hold time.
- **Availability**: if no report arrives for `5 s` the radar counts as unavailable and occupancy turns *clear* (after the hold time) instead of staying stuck (`Occupancy` is not nullable); the next report restores it.
- With no LD2410S connected, probing adds about `1 s` to boot; `CONFIG_LD2410S_ENABLED=n` skips it and leaves the UART pins free.

> **Onboard Status LED**: Driven by the `status_led` component, the onboard yellow User LED on the XIAO ESP32C6 (**`GPIO15`**, active-low, configurable via `CONFIG_STATUS_LED_GPIO`) stays **OFF during boot/init**, turns **solid ON for 3 minutes** once sensor and Matter initialization succeed (then off), blinks slowly (`1000 ms`) while the Matter commissioning window is open, and blinks rapidly (`200 ms`) when the Matter **Identify** command is triggered from Home Assistant.

---

## Project Structure

- [`main/`](main/app_main.cpp) — Application entry point (`app_main`): initializes NVS, Thread/Wi-Fi, `SensorsManager`, `StatusLed`, and `MatterManager` (restarting after 10 s if Matter fails to initialize; with no sensor detected it still starts Matter, so the device stays reachable for OTA, and probes again every 60 s, rebooting once one appears), then periodically polls sensors, passes environmental compensations (such as barometric pressure for `SCD30`), and updates Matter cluster attributes.
- [`components/common_types/`](components/common_types/include/common_types/common_types.h) — Shared [`SensorType`](components/common_types/include/common_types/common_types.h) enum (`TEMPERATURE`, `PRESSURE`, `RELATIVE_HUMIDITY`, `CARBON_DIOXIDE_CONCENTRATION`, `ILLUMINANCE`, `TVOC_CONCENTRATION`, `AIR_QUALITY`), `AirQualityLevel`, [`SensorValueRange`](components/common_types/include/common_types/common_types.h) struct (`°C`, `Pa`, `%RH`, `ppm`, `lx`, `ppb`), and `PerSensorType` containers.
- [`components/sensors_manager/`](components/sensors_manager/include/sensors_manager/sensors_manager.h) — Abstract [`Sensor`](components/sensors_manager/include/sensors_manager/sensor.h) interface, [`Tmp117`](components/sensors_manager/src/sensors/tmp117.h), [`Sht4x`](components/sensors_manager/src/sensors/sht4x.h), [`Bmp581`](components/sensors_manager/src/sensors/bmp581.h), [`Bmp280Bme280`](components/sensors_manager/src/sensors/bmp280_bme280.h), [`Scd30`](components/sensors_manager/src/sensors/scd30.h), [`Ens160`](components/sensors_manager/src/sensors/ens160.h), and [`Veml7700`](components/sensors_manager/src/sensors/veml7700.h), [`Tsl2591`](components/sensors_manager/src/sensors/tsl2591.h), [`Vl53l1x`](components/sensors_manager/src/sensors/vl53l1x.h) driver implementations, and [`SensorsManager`](components/sensors_manager/include/sensors_manager/sensors_manager.h) for probing active sensors in priority order, performing a single hardware read per active chip, and serving readings/ranges by `SensorType`.
- [`components/matter_interface/`](components/matter_interface/include/matter_interface/matter_manager.h) — [`MatterManager`](components/matter_interface/include/matter_interface/matter_manager.h) for creating Matter endpoints, scaling physical units to Matter attributes (including logarithmic lux scaling for `IlluminanceMeasurement`, and CO₂ / TVOC concentration and `AirQuality` clusters on a shared Air Quality Sensor endpoint), and updating cluster attributes with deadband + heartbeat filtering.
- [`components/ld2410s/`](components/ld2410s/include/ld2410s.h) — Hi-Link HLK-LD2410S UART driver (minimal reports, configuration commands).
- [`components/tmp117/`](components/tmp117/include/tmp117.h) — TI TMP117 / TMP119 driver on esp-idf-lib `i2cdev` (one-shot 8× averaged conversions).
- [`components/ens160/`](components/ens160/include/ens160.h) — ScioSense ENS160 driver, ported from `k0i05/esp_ens160` onto esp-idf-lib `i2cdev` so it shares the I2C bus with the other drivers.
- [`components/bmp5/`](components/bmp5/include/bmp5_i2c.h) — Bosch BMP5 SensorAPI (git submodule in `third_party/`) plus `bmp5_i2c` glue that maps its read/write/delay callbacks onto esp-idf-lib `i2cdev`.
- [`components/status_led/`](components/status_led/include/status_led/status_led.h) — Thread-safe [`StatusLed`](components/status_led/include/status_led/status_led.h) helper driven by `setReady` (solid on for 3 minutes), `setCommissioning` (slow blink) and `setIdentifying` (fast blink). It has no Matter dependency; `app_main` connects it to `MatterManager` listeners.

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

- **`Application Configuration`** ([`main/Kconfig.projbuild`](main/Kconfig.projbuild)): Sensor polling interval (`CONFIG_SENSOR_READ_DELAY_MS`, default `10000 ms`) and the XIAO ESP32-C6 antenna RF switch (`CONFIG_XIAO_RF_SWITCH`, default on: GPIO3 low powers it; `CONFIG_XIAO_EXTERNAL_ANTENNA` drives GPIO14 high for the U.FL antenna, default off = ceramic). Without it the switch is unpowered and the radio range is very short.
- **`Sensors Configuration`** ([`components/sensors_manager/Kconfig.projbuild`](components/sensors_manager/Kconfig.projbuild)): Shared I2C port (`CONFIG_SENSORS_I2C_PORT`, default `0`), SDA/SCL GPIOs (`CONFIG_SENSORS_I2C_SDA_GPIO` = `22`, `CONFIG_SENSORS_I2C_SCL_GPIO` = `23`), and the VL53L1X remaining-level calibration (`CONFIG_VL53L1X_FULL_DISTANCE_MM`, default `30`; `CONFIG_VL53L1X_LEVEL_HEIGHT_MM`, default `290`).
- **`Matter Manager Configuration`** ([`components/matter_interface/Kconfig.projbuild`](components/matter_interface/Kconfig.projbuild)): Maximum reporting interval (`CONFIG_MATTER_REPORT_MAX_INTERVAL_S`, default `180 s`) and the battery `BatChargeLevel` thresholds (`CONFIG_MATTER_BATTERY_WARNING_PERCENT`, default `20`; `CONFIG_MATTER_BATTERY_CRITICAL_PERCENT`, default `10`).
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

This project is based on the original [matter_bmp280](https://github.com/albert-gee/matter_bmp280) project by **Albert Gee** ([`@albert-gee`](https://github.com/albert-gee)). It has been extended to support multi-sensor auto-detection (`TMP119`/`TMP117`, `SHT40`/`SHT41`/`SHT45`, `BMP581`, `BME280`/`BMP280`, `SCD30`, `ENS160`, `VEML7700`, `TSL2591`, `VL53L1X`, `MAX17048`, and `LD2410S`) using the `esp-idf-lib/sht4x`, `esp-idf-lib/bmp280`, `esp-idf-lib/scd30`, `esp-idf-lib/veml7700`, and `esp-idf-lib/tsl2591`, and `grrtzm/vl53l1x_library` managed components an `i2cdev` port of Eric Gionet's `k0i05/esp_ens160` driver, and Bosch Sensortec's BMP5 SensorAPI.

---

## License

This project is licensed under the MIT License. See the [`LICENSE`](LICENSE) file for details.

---

## References

- [TI TMP119 Datasheet](https://www.ti.com/lit/ds/symlink/tmp119.pdf)
- [TI TMP117 Datasheet](https://www.ti.com/lit/ds/symlink/tmp117.pdf)
- [Sensirion SHT4x (SHT40 / SHT41 / SHT45) Datasheet](https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf)
- [Bosch BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [Bosch BMP581 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp581-ds004.pdf)
- [Bosch BMP280 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)
- [Sensirion SCD30 Datasheet](https://sensirion.com/media/documents/4EAF6AF8/61652C3C/Sensirion_CO2_Sensors_SCD30_Datasheet.pdf)
- [ScioSense ENS160 Datasheet](https://www.sciosense.com/wp-content/uploads/2023/12/ENS160-Datasheet.pdf)
- [Vishay VEML7700 Datasheet](https://www.vishay.com/docs/84286/veml7700.pdf)
- [ams TSL2591 Datasheet](https://ams.com/documents/20143/36005/TSL2591_DS000338_6-00.pdf)
- [ST VL53L1X Datasheet](https://www.st.com/resource/en/datasheet/vl53l1x.pdf)
- [Hi-Link HLK-LD2410S Serial Communication Protocol V1.00](https://www.tinytronics.nl/product_files/006002_HLK-LD2410S%20serial%20communication%20protocol-V1.00.pdf)
- [Analog Devices MAX17048 / MAX17049 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX17048-MAX17049.pdf)
- [ESP Component Registry — `esp-idf-lib/sht4x`](https://components.espressif.com/components/esp-idf-lib/sht4x)
- [ESP Component Registry — `esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280)
- [ESP Component Registry — `esp-idf-lib/scd30`](https://components.espressif.com/components/esp-idf-lib/scd30)
- [ESP Component Registry — `esp-idf-lib/veml7700`](https://components.espressif.com/components/esp-idf-lib/veml7700)
- [Espressif Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [Matter Protocol Specification](https://csa-iot.org/all-solutions/matter/)
