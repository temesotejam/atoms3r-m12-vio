#pragma once

// M5Stack AtomS3R-M12 (OV3660) pin map.
// Camera SCCB is intentionally placed on ESP32 I2C controller 1.
// BMI270 uses the internal SYS I2C bus on GPIO45 / GPIO0.

#define CAM_PIN_SIOD   12
#define CAM_PIN_SIOC    9

#define CAM_PIN_D0      3
#define CAM_PIN_D1     42
#define CAM_PIN_D2     46
#define CAM_PIN_D3     48
#define CAM_PIN_D4      4
#define CAM_PIN_D5     17
#define CAM_PIN_D6     11
#define CAM_PIN_D7     13

#define CAM_PIN_VSYNC  10
#define CAM_PIN_HREF   14
#define CAM_PIN_PCLK   40
#define CAM_PIN_XCLK   21

// AtomS3R-CAM/M12 labels GPIO18 POWER_N: LOW = camera powered.
// Do not give it to esp_camera as pin_pwdn because esp_camera assumes active-HIGH PWDN.
#define CAM_POWER_N    18
#define CAM_PIN_RESET  -1

#define IMU_PIN_SCL     0
#define IMU_PIN_SDA    45
