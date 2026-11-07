# Sensor Reporting Thresholds & Heartbeat Throttling

This document explains how `MatterManager` filters sensor readings before publishing updates to Matter endpoints, why a combined **absolute + relative threshold (`abs_threshold` + `rel_threshold`)** is used instead of a single percentage formula, and provides reference threshold values for common environmental sensors.

---

## 1. Motivation

The application polls attached sensors periodically (every `CONFIG_SENSOR_READ_DELAY_MS`, default `10,000 ms`). However, publishing every raw measurement to Matter has downsides:

1. **Home Assistant Database & SD Card Wear**: Matter temperature (`0.01 °C`) and humidity (`0.01 %RH`) clusters have high resolution. Tiny `±0.01–0.02 °C` thermal noise fluctuations would trigger a new Matter subscription report and a new SQLite recorder row in Home Assistant every 10 seconds (`8,640` writes/day per entity).
2. **Unnecessary Radio Traffic**: Suppressing sub-noise jitter reduces Wi-Fi / Thread traffic.
3. **Liveness / Drift Visibility**: Conversely, if an environment is completely stable or drifting very slowly below the threshold, Home Assistant should still receive a periodic update (`CONFIG_MATTER_REPORT_MAX_INTERVAL_S`, default `300 s`) so graphs remain current and any small accumulated drift is flushed.

---

## 2. Why a Single Percentage Formula Fails Across Different Sensors

Neither a fixed **percentage of current reading (`|Δx / x|`)** nor a fixed **percentage of full-scale range (`|Δx| / (max - min)`)** works across all physical quantities:

- **Why `% of reading` (`|Δx / x|`) fails**:
  - **Temperature (`°C`) crosses zero**: `0 °C` is an arbitrary offset, not absolute zero. Near `0 °C`, `0.5%` of the reading shrinks to `0.000 °C` (and divides by zero at `0.00 °C`), causing every read to be reported, while at `+125 °C` it grows to `0.63 °C`.
  - **Barometric Pressure (`Pa`) sits on a `~101,325 Pa` baseline**: `0.5%` of `101,325 Pa` is **`507 Pa` (`5.07 hPa`)**. Normal daily weather changes are `1–3 hPa`, so `0.5%` suppresses real weather fronts.
  - **NDIR CO₂ (`ppm`)**: At `600 ppm`, `0.5%` is `3 ppm`, which is 3–10× smaller than an NDIR sensor's own `±10–30 ppm` noise floor, causing reports on every read.

- **Why `% of full-scale range` (`max - min`) fails**:
  - **Wide-range / logarithmic sensors (Illuminance `0..120,000 lx`)**: `0.5%` of a `120,000 lx` span is **`600 lx`**. Indoors (`0–500 lx`), turning room lights on (`0 → 300 lx`) or off (`300 → 0 lx`) would **never** exceed `600 lx` and would never trigger an immediate report.
  - **Precision temperature sensors with wide industrial ranges (TMP119 `-55..+150 °C`, SHT45 `-40..+125 °C`)**: `0.5%` of a `205 °C` span is **`1.03 °C`**, throwing away `±0.08–0.10 °C` sensor precision.

---

## 3. How the Combined Threshold & Heartbeat Mechanism Works

Each sensor driver specifies `abs_threshold` and `rel_threshold` per `SensorType` inside [`SensorValueRange`](../components/common_types/include/common_types/common_types.h):

```cpp
struct SensorValueRange {
    float min;
    float max;
    float abs_threshold{0.0F}; // Minimum absolute change to report immediately (same unit as min/max)
    float rel_threshold{0.0F}; // Minimum relative change as a fraction of |last_reported_value|
};
```

On each polling cycle, [`shouldSkipReport()`](../components/matter_interface/src/matter_manager.cpp) in `MatterManager` evaluates:

1. **Max Reporting Interval (`CONFIG_MATTER_REPORT_MAX_INTERVAL_S`)**:
   - Measured with `esp_timer_get_time()` against `last_report_time_us`.
   - When `(now_us - last_report_time_us) >= CONFIG_MATTER_REPORT_MAX_INTERVAL_S * 1,000,000`, a report is forced regardless of how little the value changed, and `last_report_time_us` is updated.
2. **Availability Transitions**:
   - The very first valid reading after boot (or any transition between a valid value and `std::nullopt` on sensor error) is reported immediately.
3. **Combined Deadband Formula**:
   $$\text{report immediately if } |x_{\text{new}} - x_{\text{last}}| \ge \max\!\big(\text{abs\_threshold},\; \text{rel\_threshold} \times |x_{\text{last}}|\big)$$
   - **Linear / offset quantities** (Temperature, Pressure, Humidity) set `abs_threshold > 0` and `rel_threshold = 0`.
   - **Wide-dynamic-range / ratio quantities** (CO₂, Illuminance) set **both** `abs_threshold > 0` (to block baseline noise at the low end) and `rel_threshold > 0` (to scale proportionally at high readings).

---

## 4. Reference Table for Sensor Drivers

Thresholds are chosen using the rule of thumb:
$$\text{threshold} \approx \max\!\big(2.5 \times \text{Repeatability}_{3\sigma},\; 1 \times \text{Matter LSB}\big)$$

| Sensor | `SensorType` | Datasheet Range (`min .. max`) | Resolution (1 LSB) | Repeatability / Noise Floor ($3\sigma$ or RMS) | Datasheet Accuracy | Matter Cluster Resolution | `abs_threshold` | `rel_threshold` |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **TI TMP119** | **Temperature** | `-55 .. +150 °C` | `0.0078 °C` | `±0.008 °C` (1 LSB) / `±0.003 °C` (averaged) | `±0.08 °C` (`-20..50 °C`) / `±0.15 °C` (full range) | `0.01 °C` | **`0.05 °C`** | **`0.0`** |
| **Sensirion SHT45** | **Temperature** | `-40 .. +125 °C` | `0.01 °C` | **`0.04 °C`** ($3\sigma$, high rep.) | `±0.10 °C` (`0..75 °C`) / `±0.25 °C` (extremes) | `0.01 °C` | **`0.08 °C`** *(or `0.10 °C`)* | **`0.0`** |
| **Sensirion SHT45** | **Humidity** | `0 .. 100 %RH` | `0.01 %RH` | **`0.08 %RH`** ($3\sigma$, high rep.) | `±1.0 %RH` (`10..90 %RH`) / `±1.5 %RH` (extremes) | `0.01 %RH` | **`0.20 %RH`** | **`0.0`** |
| **Bosch BMP581** | **Temperature** | `-40 .. +85 °C` | `0.000015 °C` | `0.002 °C` RMS (`~0.006 °C` $3\sigma$) | `±0.50 °C` | `0.01 °C` | **`0.10 °C`** | **`0.0`** |
| **Bosch BMP581** | **Pressure** | `30,000 .. 125,000 Pa` | `0.015 Pa` | `0.08 Pa` RMS (`~0.24 Pa` $3\sigma$) | `±3 Pa` relative / `±30 Pa` absolute | **`100 Pa`** (`0.1 kPa` = `1 hPa`)¹ | **`50 Pa`** (`0.5 hPa`)¹ | **`0.0`** |
| **Bosch BME280 / BMP280** | **Temperature** | `-40 .. +85 °C` | `0.01 °C` | `0.005 °C` RMS (`~0.015 °C` $3\sigma$) | `±0.5 °C` (`25 °C`) / `±1.0 °C` (`0..65 °C`) | `0.01 °C` | **`0.10 °C`** | **`0.0`** |
| **Bosch BME280 / BMP280** | **Pressure** | `30,000 .. 110,000 Pa` | `0.18 Pa` | `0.2 Pa` RMS (IIR on) / `1.3 Pa` RMS (IIR off, `~4 Pa` $3\sigma$) | `±12 Pa` relative / `±100 Pa` absolute | **`100 Pa`** (`0.1 kPa` = `1 hPa`)¹ | **`50 Pa`** (`0.5 hPa`)¹ | **`0.0`** |
| **Bosch BME280** | **Humidity** | `0 .. 100 %RH` | `0.008 %RH` | `0.07 %RH` RMS (**`~0.21 %RH` $3\sigma$**) | `±3.0 %RH` (`±1 %RH` hyst.) | `0.01 %RH` | **`0.50 %RH`** | **`0.0`** |
| **Sensirion SCD-30** | **CO₂** | `400 .. 40,000 ppm` / *(cal. `400..10,000`)* | `1 ppm` | **`±10 ppm`** ($3\sigma$ at `400–2,000 ppm`, scales with `[CO₂]`) | **`±(30 ppm + 3% of reading)`** | `1 ppm` | **`20 ppm`** | **`0.02` (`2%`)** *(or `3%`)* |
| **Sensirion SCD-30** | **Temperature** *(onboard SHT31)* | `-40 .. +70 °C` | `0.01 °C` | **`0.10 °C`** ($3\sigma$, affected by NDIR lamp heating) | `±(0.4 °C + 0.023 × (T − 25 °C))` | `0.01 °C` | **`0.20 °C`** | **`0.0`** |
| **Sensirion SCD-30** | **Humidity** *(onboard SHT31)* | `0 .. 100 %RH` | `0.01 %RH` | **`0.20 %RH`** ($3\sigma$) | `±3.0 %RH` | `0.01 %RH` | **`0.50 %RH`** | **`0.0`** |
| **ScioSense ENS160** | **eCO₂** | `400 .. 65,000 ppm` | `1 ppm` | Algorithm output, updated every `1 s`; step changes of `~10–20 ppm` | Estimated from VOCs (not NDIR); relative trend only | `1 ppm` | **`20 ppm`** | **`0.02` (`2%`)** |
| **ScioSense ENS160** | **TVOC** | `0 .. 65,000 ppb` | `1 ppb` | Algorithm output, updated every `1 s` | Relative trend; baseline drifts during the first hour of operation | `1 ppb` | **`10 ppb`** | **`0.05` (`5%`)** |
| **ScioSense ENS160** | **Air Quality** *(UBA index 1–5)* | `1 .. 5` (mapped to Matter `Good .. VeryPoor`) | `1` level | — | — | `1` level | **`1` level** | **`0.0`** |
| **Vishay VEML7700** | **Illuminance** | `0 .. 120,000 lx` | `0.0036 lx` (high gain) to `1.84 lx` (low gain) | `±1–2 LSB` dark noise + `~1–2%` artificial light flicker | `±10% .. ±15%` | Logarithmic (`10000 × log10(lx) + 1`) | **`1.0 lx`** | **`0.03` (`3%`)** to **`0.05` (`5%`)** |
| **ams TSL2591** | **Illuminance** | `0.000188 .. 88,000 lx` | `~0.0002 lx` (max gain, 600 ms) to `~2.3 lx` (low gain, 100 ms) | `±1–2` counts dark noise; auto-ranged gain/integration | `±10% .. ±20%` (IR-compensated, light-source dependent) | Logarithmic (`10000 × log10(lx) + 1`) | **`1.0 lx`** | **`0.03` (`3%`)** |
| **ST VL53L1X** | **Remaining Level** *(from distance)* | `0 .. 100 %` (distance `40 .. 4,000 mm`, configured full..empty span) | `1 mm` | few mm (median of 5 samples, 4×4 ROI) | `±` few % of span (target/ambient dependent) | `1 %` (HEPA `Condition`, whole percent) | **`1 %`** | **`0.0`** |
| **Analog Devices MAX17048** | **Battery Percent** | `0 .. 100 %` | `1/256 %` | ModelGauge output, smooth | Depends on battery model / cycles (uncalibrated) | `0.5 %` | **`1 %`** | **`0.0`** |
| **Analog Devices MAX17048** | **Battery Voltage** | `0 .. 5.12 V` | `78.125 µV` | `~±1 mV` | `±7.5 mV` | `1 mV` | **`10 mV`** | **`0.0`** |
| **Analog Devices MAX17048** | **Battery Charge State** *(from charge rate)* | `charging / full / not charging` | `1` state | — | — | `1` state | **`1` state** | **`0.0`** |
| **Hi-Link HLK-LD2410S** | **Occupancy** *(pushed on change)* | `clear / occupied` | `1` state | — | — | `1` state | **every change** *(not polled; HoldTime off-delay)* | **`0.0`** |

> **¹ Note on Pressure (`50 Pa` vs `100 Pa`):**
> Matter's `PressureMeasurementCluster` `MeasuredValue` attribute is `int16_t` in **`0.1 kPa` (`100 Pa = 1 hPa`)** units. Setting `abs_threshold = 50 Pa` (`0.5 hPa`, half a Matter LSB) ensures that any `0.5–1.0 hPa` change that crosses a rounding boundary updates Matter immediately, while ignoring `< 50 Pa` fluctuations that round to the same Matter integer anyway.
