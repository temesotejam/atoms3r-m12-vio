# AtomS3R-M12 VIO

Small, self-contained visual-inertial odometry research firmware for **M5Stack AtomS3R-M12**.

## Goal

Use only the sensors already inside AtomS3R-M12:

- OV3660 camera
- BMI270 IMU
- ESP32-S3 + PSRAM

to estimate the module's relative motion without external anchors or markers.

## Current stage: v0.1 foundation

This first version intentionally prioritizes hardware validation and deterministic data flow before a full monocular VIO solver.

Implemented:

- AtomS3R-M12 camera bring-up (M12 pin map, camera power handling, separate SCCB I2C port)
- BMI270 acquisition on the internal I2C bus
- 200 Hz IMU attitude propagation
- startup gyro-bias calibration
- QVGA grayscale/RGB565 camera capture
- lightweight sparse block-matching optical flow
- visual-flow assisted stationary detection / zero-velocity update
- experimental short-term inertial position and velocity output
- CSV pose telemetry over USB serial
- GitHub Actions build
- GitHub Pages WebSerial flasher

The position fields in v0.1 are **experimental inertial dead-reckoning values**. They are not yet corrected by a full visual translation/scale estimator. The camera is already used for motion/stationary detection; the next stage is true monocular visual-inertial translation fusion.

## Web flasher

After the Pages workflow is deployed:

https://temesotejam.github.io/atoms3r-m12-vio/

Use Chrome or Edge on desktop and connect the AtomS3R-M12 by USB.

## Serial output

115200 baud.

Pose rows:

```text
POSE,t_us,px,py,pz,vx,vy,vz,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,flow_x,flow_y,tracks,stationary
```

Commands:

- `STATUS`
- `RESET_POSE`
- `CSV 1`
- `CSV 0`

## First hardware test

1. Flash from GitHub Pages.
2. Keep the unit stationary for the first ~2.5 s.
3. Open the serial monitor.
4. Confirm `CAM OK`, `IMU OK`, and regular `POSE` lines.
5. Rotate the unit and verify Roll/Pitch/Yaw respond.
6. Move the camera in front of a textured scene and confirm `tracks` is typically > 10 and flow changes.
7. Return it to rest and confirm `stationary=1` and velocity is driven back toward zero.

## Why this order

A reliable VIO implementation depends first on:

1. camera/IMU timestamps,
2. stable camera capture,
3. correct IMU axes and units,
4. repeatable feature tracking,
5. measured CPU and memory headroom.

Once these are verified on the actual M12 hardware, the repository can move to the full metric monocular VIO update without guessing about the camera timing or ESP32-S3 budget.
