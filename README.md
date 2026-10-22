# Matter-Based BME280 / BMP280 Temperature, Pressure, and Humidity Sensor

This project implements a Matter-compatible **Temperature**, **Pressure**, and **Relative Humidity** sensor on ESP32 microcontrollers (such as ESP32-C6, ESP32-H2, or ESP32) using **ESP-Matter** and the [`esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280) component from the ESP Component Registry.

It supports both:

- **BME280**: Temperature, Pressure, and Relative Humidity
- **BMP280**: Temperature and Pressure

---

## Documentation

- [**Home Assistant Commissioning & QR Code Guide**](docs/home_assistant_commissioning.md) — How to pair the device over Matter-over-Wi-Fi (via the Home Assistant Companion App or pure Wi-Fi) and how to use **ESP Launchpad** or **`esp-matter-mfg-tool`** to simplify flashing and QR code generation.
- [**Matter OTA Firmware Updates Guide**](docs/ota_firmware_updates.md) — How to enable Matter OTA Requestor, build `.ota` upgrade images, and push firmware updates via Home Assistant or Espressif portals.

---

## Prerequisites

### Hardware

- **Seeed Studio XIAO ESP32C6** (or ESP32-H2 / ESP32 development board)
- **Adafruit BME280** Temperature, Humidity, and Barometric Pressure Sensor module (or BMP280 module) over I2C
- Breadboard and connecting wires
- 3.3V power supply

### Software

- **ESP-IDF >= 6.1.0**
- **ESP-Matter**
- **`esp-idf-lib/bmp280`** (automatically fetched via the IDF Component Manager from [`main/idf_component.yml`](main/idf_component.yml))

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

## Wiring (Seeed Studio XIAO ESP32C6 + Adafruit BME280)

Connect the **Adafruit BME280** module to the **Seeed Studio XIAO ESP32C6** using the default hardware I2C pins:

| Adafruit BME280 Pin | XIAO ESP32C6 Pin | ESP32-C6 GPIO | Notes |
| --- | --- | --- | --- |
| **VIN** (or **3Vo**) | **3V3** | — | 3.3V power supply |
| **GND** | **GND** | — | Common ground |
| **SDI** (SDA) | **D4** (SDA) | **GPIO22** | Configurable via `CONFIG_BMP280_I2C_SDA_GPIO` |
| **SCK** (SCL) | **D5** (SCL) | **GPIO23** | Configurable via `CONFIG_BMP280_I2C_SCL_GPIO` |
| **SDO** | Unconnected | — | Pulled high on Adafruit module → default I2C address **`0x77`**. Connect to `GND` for `0x76`. |

> **Auto-Detection**: At boot, the firmware checks the sensor's chip ID (`0x60` for BME280, `0x58` for BMP280). If a **BMP280** is detected, the Matter Relative Humidity endpoint is automatically omitted and only Temperature and Pressure are exposed.
> **Onboard Status LED**: The onboard yellow User LED on the XIAO ESP32C6 (**`GPIO15`**, active-low, configurable via `CONFIG_STATUS_LED_GPIO`) stays **OFF during boot/init**, turns **solid ON** during normal operation once sensor and Matter initialization succeed, blinks slowly (`1000 ms`) while the Matter commissioning window is open, and blinks rapidly (`200 ms`) when the Matter **Identify** command is triggered from Home Assistant.

---

## Build and Flash

### Step 1. Set Target Chip

```bash
idf.py set-target esp32c6   # or esp32 / esp32h2
```

### Step 2. Configure Sensor Settings (Optional)

Sensor I2C address (`0x77` default for Adafruit BME280, or `0x76`), SDA/SCL GPIOs (`22`/`23`), I2C port (`0`), and polling interval (`10000 ms`) can be customized in **`BMP280 / BME280 Sensor Configuration`**:

```bash
idf.py menuconfig
```

### Step 3. Build, Flash, and Monitor

```bash
idf.py -p <PORT> flash monitor
```

Press `CTRL+]` to exit the serial monitor.

#### Optional: Building Separate Wi-Fi and Thread Firmwares

Because [`main/app_main.cpp`](main/app_main.cpp) supports both Matter-over-Wi-Fi and Matter-over-Thread via `#if CHIP_DEVICE_CONFIG_ENABLE_THREAD`, you can build both variants into separate build directories without modifying source code:

- **Matter over Wi-Fi** (into `build_wifi/`):

  ```bash
  idf.py -B build_wifi -DSDKCONFIG_DEFAULTS="sdkconfig.defaults" set-target esp32c6
  idf.py -B build_wifi build
  idf.py -B build_wifi -p <PORT> flash monitor
  ```

- **Matter over Thread** (into `build_thread/`):

  ```bash
  idf.py -B build_thread -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32h2" set-target esp32c6
  idf.py -B build_thread build
  idf.py -B build_thread -p <PORT> flash monitor
  ```

Once running, follow the [**Home Assistant Commissioning Guide**](docs/home_assistant_commissioning.md) to add the sensor to Home Assistant.

---

## Acknowledgements

This project is based on the original [matter_bmp280](https://github.com/albert-gee/matter_bmp280) project by **Albert Gee** ([`@albert-gee`](https://github.com/albert-gee)). It has been extended to use the `esp-idf-lib/bmp280` managed component and expose BME280 Relative Humidity over Matter.

---

## License

This project is licensed under the MIT License. See the [`LICENSE`](LICENSE) file for details.

---

## References

- [BME280 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)
- [BMP280 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)
- [ESP Component Registry — `esp-idf-lib/bmp280`](https://components.espressif.com/components/esp-idf-lib/bmp280)
- [Espressif Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/)
- [Matter Protocol Specification](https://csa-iot.org/all-solutions/matter/)
