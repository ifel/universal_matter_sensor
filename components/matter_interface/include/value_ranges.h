#ifndef VALUE_RANGES_H
#define VALUE_RANGES_H

// BMP280 / BME280 Sensor Ranges
// Temperature range
constexpr float bmp_280_temp_min_deg_celsius = -40.0F;
constexpr float bmp_280_temp_max_deg_celsius = 85.0F;

// Pressure range
constexpr float bmp_280_press_min_pa = 30000.0F;
constexpr float bmp_280_press_max_pa = 110000.0F;

// Relative Humidity range
constexpr float bmp_280_humi_min_percent = 0.0F;
constexpr float bmp_280_humi_max_percent = 100.0F;

#endif // VALUE_RANGES_H
