#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "SparkFun_BMI270_Arduino_Library.h"

#include "board_atoms3r_m12.h"
#include "mahony_imu.h"

namespace {

constexpr float G0 = 9.80665f;
constexpr const char* FW_VERSION = "v0.4-pyramid";
constexpr uint32_t IMU_PERIOD_US = 5000;       // 200 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 50;   // 20 Hz

// Capture and track directly at QQVGA. The OV3660 driver uses a faster
// non-JPEG PLL regime below QVGA, raising sensor-side frame rate from the
// ~10 fps QVGA regime toward ~17.8 fps.
constexpr int CAP_W = 160;
constexpr int CAP_H = 120;
constexpr int CAM_W = 160;
constexpr int CAM_H = 120;
constexpr int MAX_FEATURES = 48;
constexpr int GRID_X = 8;
constexpr int GRID_Y = 6;
constexpr int PATCH_R = 2;
constexpr int PYR_W = CAM_W / 2;
constexpr int PYR_H = CAM_H / 2;
constexpr int COARSE_SEARCH_R = 8;   // +/-16 px equivalent at full resolution
constexpr int REFINE_SEARCH_R = 3;   // full-resolution refinement around coarse result

struct FlowState {
  float dx = 0.0f;
  float dy = 0.0f;
  float fps = 0.0f;
  float proc_ms = 0.0f;
  uint16_t detected = 0;
  uint16_t tracks = 0;
  uint16_t saturated = 0;
  uint64_t t_us = 0;
  bool valid = false;
};

struct PoseState {
  Vec3f p{};
  Vec3f v{};
  Vec3f linear_a{};
  Quatf q{};
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float gyro_norm = 0.0f;
  float accel_norm_g = 1.0f;
  bool stationary = false;
  uint64_t t_us = 0;
};

struct Pt {
  int16_t x;
  int16_t y;
};

BMI270 g_imu;
MahonyImu g_att;

portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;
FlowState g_flow;
PoseState g_pose;

float g_gyro_bias_x = 0.0f;
float g_gyro_bias_y = 0.0f;
float g_gyro_bias_z = 0.0f;
Vec3f g_world_accel_bias{};

bool g_camera_ok = false;
bool g_camera_rgb565 = false;
bool g_imu_ok = false;
bool g_csv = true;

uint8_t* g_gray_a = nullptr;
uint8_t* g_gray_b = nullptr;
uint8_t* g_half_a = nullptr;
uint8_t* g_half_b = nullptr;

inline uint8_t grayAt(const uint8_t* img, int x, int y) {
  return img[y * CAM_W + x];
}

float medianSmall(float* a, int n) {
  for (int i = 1; i < n; ++i) {
    const float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) {
      a[j + 1] = a[j];
      --j;
    }
    a[j + 1] = v;
  }
  if (n <= 0) return 0.0f;
  return (n & 1) ? a[n/2] : 0.5f * (a[n/2 - 1] + a[n/2]);
}

int detectFeatures(const uint8_t* img, Pt* out) {
  int count = 0;
  const int cell_w = CAM_W / GRID_X;
  const int cell_h = CAM_H / GRID_Y;

  for (int gy = 0; gy < GRID_Y && count < MAX_FEATURES; ++gy) {
    for (int gx = 0; gx < GRID_X && count < MAX_FEATURES; ++gx) {
      const int x0 = max(8, gx * cell_w + 4);
      const int x1 = min(CAM_W - 8, (gx + 1) * cell_w - 4);
      const int y0 = max(8, gy * cell_h + 4);
      const int y1 = min(CAM_H - 8, (gy + 1) * cell_h - 4);

      int best_x = -1, best_y = -1, best_score = 0;
      for (int y = y0; y < y1; y += 3) {
        for (int x = x0; x < x1; x += 3) {
          const int dx = abs((int)grayAt(img, x + 1, y) - (int)grayAt(img, x - 1, y));
          const int dy = abs((int)grayAt(img, x, y + 1) - (int)grayAt(img, x, y - 1));
          const int d1 = abs((int)grayAt(img, x + 1, y + 1) - (int)grayAt(img, x - 1, y - 1));
          const int d2 = abs((int)grayAt(img, x + 1, y - 1) - (int)grayAt(img, x - 1, y + 1));
          const int score = min(dx + d1/2, dy + d2/2);
          if (score > best_score) {
            best_score = score;
            best_x = x;
            best_y = y;
          }
        }
      }
      if (best_score >= 18 && best_x >= 0) {
        out[count++] = {(int16_t)best_x, (int16_t)best_y};
      }
    }
  }
  return count;
}

int patchSadGeneric(const uint8_t* a, const uint8_t* b, int w,
                    int x0, int y0, int x1, int y1) {
  int sad = 0;
  for (int py = -PATCH_R; py <= PATCH_R; ++py) {
    for (int px = -PATCH_R; px <= PATCH_R; ++px) {
      const int va = a[(y0 + py) * w + (x0 + px)];
      const int vb = b[(y1 + py) * w + (x1 + px)];
      sad += abs(va - vb);
    }
  }
  return sad;
}

void downsampleHalf(const uint8_t* src, uint8_t* dst) {
  for (int y = 0; y < PYR_H; ++y) {
    const int sy = 2 * y;
    for (int x = 0; x < PYR_W; ++x) {
      const int sx = 2 * x;
      const int i0 = sy * CAM_W + sx;
      const int sum = src[i0] + src[i0 + 1]
                    + src[i0 + CAM_W] + src[i0 + CAM_W + 1];
      dst[y * PYR_W + x] = (uint8_t)((sum + 2) >> 2);
    }
  }
}

FlowState estimateFlow(const uint8_t* prev, const uint8_t* curr,
                       const uint8_t* prev_half, const uint8_t* curr_half,
                       uint64_t t_us, float fps) {
  const uint64_t proc_start = esp_timer_get_time();
  Pt features[MAX_FEATURES];
  float dxs[MAX_FEATURES];
  float dys[MAX_FEATURES];

  const int n = detectFeatures(prev, features);
  int good = 0;
  int saturated = 0;

  constexpr int PATCH_PIXELS = (PATCH_R * 2 + 1) * (PATCH_R * 2 + 1);

  for (int i = 0; i < n; ++i) {
    const int x = features[i].x;
    const int y = features[i].y;
    const int hx = x >> 1;
    const int hy = y >> 1;

    if (hx < PATCH_R + 1 || hx >= PYR_W - PATCH_R - 1 ||
        hy < PATCH_R + 1 || hy >= PYR_H - PATCH_R - 1) {
      continue;
    }

    int coarse_best = 1 << 30;
    int coarse_dx = 0, coarse_dy = 0;

    for (int dy = -COARSE_SEARCH_R; dy <= COARSE_SEARCH_R; ++dy) {
      for (int dx = -COARSE_SEARCH_R; dx <= COARSE_SEARCH_R; ++dx) {
        const int xx = hx + dx;
        const int yy = hy + dy;
        if (xx < PATCH_R + 1 || xx >= PYR_W - PATCH_R - 1 ||
            yy < PATCH_R + 1 || yy >= PYR_H - PATCH_R - 1) {
          continue;
        }
        const int sad = patchSadGeneric(prev_half, curr_half, PYR_W, hx, hy, xx, yy);
        if (sad < coarse_best) {
          coarse_best = sad;
          coarse_dx = dx;
          coarse_dy = dy;
        }
      }
    }

    if (coarse_best == (1 << 30)) continue;
    if (abs(coarse_dx) == COARSE_SEARCH_R || abs(coarse_dy) == COARSE_SEARCH_R) {
      ++saturated;
    }

    const int pred_x = x + 2 * coarse_dx;
    const int pred_y = y + 2 * coarse_dy;

    int best_sad = 1 << 30;
    int second_sad = 1 << 30;
    int best_x = pred_x, best_y = pred_y;

    for (int dy = -REFINE_SEARCH_R; dy <= REFINE_SEARCH_R; ++dy) {
      for (int dx = -REFINE_SEARCH_R; dx <= REFINE_SEARCH_R; ++dx) {
        const int xx = pred_x + dx;
        const int yy = pred_y + dy;
        if (xx < PATCH_R + 1 || xx >= CAM_W - PATCH_R - 1 ||
            yy < PATCH_R + 1 || yy >= CAM_H - PATCH_R - 1) {
          continue;
        }
        const int sad = patchSadGeneric(prev, curr, CAM_W, x, y, xx, yy);
        if (sad < best_sad) {
          second_sad = best_sad;
          best_sad = sad;
          best_x = xx;
          best_y = yy;
        } else if (sad < second_sad) {
          second_sad = sad;
        }
      }
    }

    if (best_sad == (1 << 30)) continue;

    const float mean_sad = (float)best_sad / PATCH_PIXELS;
    // Slightly relaxed from v0.3. The later VIO stage will use robust
    // geometric outlier rejection; at this stage retaining tracks is more useful.
    const bool unique = second_sad > best_sad + 20;
    if (mean_sad < 30.0f && unique) {
      dxs[good] = (float)(best_x - x);
      dys[good] = (float)(best_y - y);
      ++good;
    }
  }

  FlowState f;
  f.t_us = t_us;
  f.fps = fps;
  f.detected = n;
  f.tracks = good;
  f.saturated = saturated;
  f.valid = good >= 6;
  if (f.valid) {
    f.dx = medianSmall(dxs, good);
    f.dy = medianSmall(dys, good);
  }
  f.proc_ms = (float)(esp_timer_get_time() - proc_start) * 1e-3f;
  return f;
}

bool frameToGray(const camera_fb_t* fb, uint8_t* dst) {
  if (!fb || fb->width != CAP_W || fb->height != CAP_H) return false;

  if (fb->format == PIXFORMAT_GRAYSCALE &&
      fb->len >= (size_t)CAP_W * CAP_H) {
    memcpy(dst, fb->buf, (size_t)CAP_W * CAP_H);
    return true;
  }

  if (fb->format == PIXFORMAT_RGB565 &&
      fb->len >= (size_t)CAP_W * CAP_H * 2) {
    for (int i = 0; i < CAP_W * CAP_H; ++i) {
      const uint16_t p = (uint16_t)fb->buf[2*i] |
                         ((uint16_t)fb->buf[2*i + 1] << 8);
      const int r = ((p >> 11) & 0x1F) << 3;
      const int g = ((p >> 5) & 0x3F) << 2;
      const int b = (p & 0x1F) << 3;
      dst[i] = (uint8_t)((77*r + 150*g + 29*b) >> 8);
    }
    return true;
  }
  return false;
}

void cameraPower(bool on) {
  pinMode(CAM_POWER_N, OUTPUT);
  digitalWrite(CAM_POWER_N, on ? LOW : HIGH);
}

camera_config_t cameraConfig(pixformat_t fmt) {
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_1;
  c.ledc_timer = LEDC_TIMER_1;
  c.pin_d0 = CAM_PIN_D0;
  c.pin_d1 = CAM_PIN_D1;
  c.pin_d2 = CAM_PIN_D2;
  c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4;
  c.pin_d5 = CAM_PIN_D5;
  c.pin_d6 = CAM_PIN_D6;
  c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = -1;
  c.pin_reset = CAM_PIN_RESET;
  c.sccb_i2c_port = 1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = fmt;
  c.frame_size = FRAMESIZE_QQVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return c;
}

bool tryCamera(pixformat_t fmt) {
  camera_config_t c = cameraConfig(fmt);
  const esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    Serial.printf("[CAM] init fmt=%d failed: 0x%x\n", (int)fmt, (unsigned)e);
    esp_camera_deinit();
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) s->set_framesize(s, FRAMESIZE_QQVGA);
  g_camera_rgb565 = (fmt == PIXFORMAT_RGB565);
  return true;
}

bool initCamera() {
  cameraPower(false);
  delay(80);
  cameraPower(true);
  delay(600);

  Serial.printf("[CAM] PSRAM: %s, size=%u\n", psramFound() ? "OK" : "NOT FOUND", (unsigned)ESP.getPsramSize());

  if (tryCamera(PIXFORMAT_GRAYSCALE)) return true;

  cameraPower(false);
  delay(80);
  cameraPower(true);
  delay(600);
  if (tryCamera(PIXFORMAT_RGB565)) return true;

  return false;
}

void correctImuAxes(float rax, float ray, float raz,
                    float rgx, float rgy, float rgz,
                    float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz) {
  // Match M5Unified's AtomS3R orientation correction:
  // swap X/Y and invert the resulting Y axis.
  ax = ray;
  ay = -rax;
  az = raz;
  gx = rgy;
  gy = -rgx;
  gz = rgz;
}

bool readImu(float& ax, float& ay, float& az, float& gx_dps, float& gy_dps, float& gz_dps) {
  if (!g_imu_ok) return false;
  g_imu.getSensorData();
  correctImuAxes(
    g_imu.data.accelX, g_imu.data.accelY, g_imu.data.accelZ,
    g_imu.data.gyroX,  g_imu.data.gyroY,  g_imu.data.gyroZ,
    ax, ay, az, gx_dps, gy_dps, gz_dps
  );
  return true;
}

bool initImu() {
  Wire.begin(IMU_PIN_SDA, IMU_PIN_SCL);
  Wire.setClock(400000);

  for (int i = 0; i < 5; ++i) {
    const int8_t e = g_imu.beginI2C(BMI2_I2C_PRIM_ADDR);
    if (e == BMI2_OK) {
      g_imu_ok = true;
      return true;
    }
    delay(150);
  }
  return false;
}

bool calibrateGyro() {
  constexpr int N = 400;  // ~2 s at 200 Hz
  double sx=0, sy=0, sz=0;
  float ax=0,ay=0,az=1,gx=0,gy=0,gz=0;
  int good = 0;

  Serial.println("[IMU] Keep the unit stationary: gyro bias calibration...");
  for (int i = 0; i < N; ++i) {
    if (readImu(ax,ay,az,gx,gy,gz)) {
      sx += gx; sy += gy; sz += gz;
      ++good;
    }
    delay(5);
  }
  if (good < N/2) return false;

  g_gyro_bias_x = (float)(sx / good);
  g_gyro_bias_y = (float)(sy / good);
  g_gyro_bias_z = (float)(sz / good);

  if (readImu(ax,ay,az,gx,gy,gz)) {
    g_att.setFromAccel(ax,ay,az);
  }

  Serial.printf("[IMU] bias dps: %.4f %.4f %.4f\n", g_gyro_bias_x, g_gyro_bias_y, g_gyro_bias_z);
  return true;
}

void resetPose() {
  portENTER_CRITICAL(&g_state_mux);
  g_pose.p = {};
  g_pose.v = {};
  g_world_accel_bias = {};
  portEXIT_CRITICAL(&g_state_mux);
  Serial.println("[POSE] position/velocity reset");
}

void imuTask(void*) {
  uint64_t last_us = esp_timer_get_time();

  for (;;) {
    const uint64_t now = esp_timer_get_time();
    const float dt = (float)(now - last_us) * 1e-6f;
    if ((now - last_us) < IMU_PERIOD_US) {
      delayMicroseconds(200);
      continue;
    }
    last_us = now;

    float ax,ay,az,gx_dps,gy_dps,gz_dps;
    if (!readImu(ax,ay,az,gx_dps,gy_dps,gz_dps)) {
      vTaskDelay(1);
      continue;
    }

    gx_dps -= g_gyro_bias_x;
    gy_dps -= g_gyro_bias_y;
    gz_dps -= g_gyro_bias_z;

    constexpr float D2R = (float)M_PI / 180.0f;
    const float gx = gx_dps * D2R;
    const float gy = gy_dps * D2R;
    const float gz = gz_dps * D2R;

    g_att.update(gx,gy,gz,ax,ay,az,dt);

    const float accel_norm = sqrtf(ax*ax + ay*ay + az*az);
    const float gyro_norm = sqrtf(gx*gx + gy*gy + gz*gz);

    FlowState flow;
    portENTER_CRITICAL(&g_state_mux);
    flow = g_flow;
    portEXIT_CRITICAL(&g_state_mux);

    const float flow_mag = sqrtf(flow.dx*flow.dx + flow.dy*flow.dy);
    const bool flow_fresh = flow.valid && now >= flow.t_us && (now - flow.t_us) < 250000ULL;
    const bool visual_still = flow_fresh && flow_mag < 0.45f;
    const bool imu_still = gyro_norm < 0.060f && fabsf(accel_norm - 1.0f) < 0.045f;
    // ZUPT is asserted only when BOTH IMU and a recent valid visual flow agree on stillness.
    // This avoids falsely declaring stillness when visual tracking is lost during motion.
    const bool stationary = imu_still && visual_still;

    Vec3f body_a_ms2{ax * G0, ay * G0, az * G0};
    Vec3f world_specific = g_att.rotateBodyToWorld(body_a_ms2);
    Vec3f lin{
      world_specific.x - g_world_accel_bias.x,
      world_specific.y - g_world_accel_bias.y,
      world_specific.z - G0 - g_world_accel_bias.z
    };

    if (stationary) {
      constexpr float beta = 0.015f;
      g_world_accel_bias.x = (1.0f-beta)*g_world_accel_bias.x + beta*world_specific.x;
      g_world_accel_bias.y = (1.0f-beta)*g_world_accel_bias.y + beta*world_specific.y;
      g_world_accel_bias.z = (1.0f-beta)*g_world_accel_bias.z + beta*(world_specific.z - G0);
      lin = {};
    }

    if (fabsf(lin.x) < 0.06f) lin.x = 0;
    if (fabsf(lin.y) < 0.06f) lin.y = 0;
    if (fabsf(lin.z) < 0.06f) lin.z = 0;

    portENTER_CRITICAL(&g_state_mux);

    if (stationary) {
      g_pose.v = {};
    } else if (dt > 0.0f && dt < 0.03f) {
      g_pose.p.x += g_pose.v.x*dt + 0.5f*lin.x*dt*dt;
      g_pose.p.y += g_pose.v.y*dt + 0.5f*lin.y*dt*dt;
      g_pose.p.z += g_pose.v.z*dt + 0.5f*lin.z*dt*dt;
      g_pose.v.x += lin.x*dt;
      g_pose.v.y += lin.y*dt;
      g_pose.v.z += lin.z*dt;
    }

    g_pose.linear_a = lin;
    g_pose.q = g_att.q();
    g_att.eulerDeg(g_pose.roll, g_pose.pitch, g_pose.yaw);
    g_pose.gyro_norm = gyro_norm;
    g_pose.accel_norm_g = accel_norm;
    g_pose.stationary = stationary;
    g_pose.t_us = now;
    portEXIT_CRITICAL(&g_state_mux);
  }
}

void cameraTask(void*) {
  if (!g_camera_ok || !g_gray_a || !g_gray_b || !g_half_a || !g_half_b) {
    vTaskDelete(nullptr);
    return;
  }

  uint8_t* prev = g_gray_a;
  uint8_t* curr = g_gray_b;
  uint8_t* prev_half = g_half_a;
  uint8_t* curr_half = g_half_b;
  bool have_prev = false;
  uint64_t last_t = 0;

  for (;;) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      delay(5);
      continue;
    }
    const uint64_t now = esp_timer_get_time();
    const bool ok = frameToGray(fb, curr);
    esp_camera_fb_return(fb);

    if (!ok) {
      Serial.println("[CAM] unsupported frame format/size");
      delay(100);
      continue;
    }

    downsampleHalf(curr, curr_half);

    if (have_prev) {
      const float fps = (last_t > 0 && now > last_t) ? 1e6f / (float)(now - last_t) : 0.0f;
      FlowState f = estimateFlow(prev, curr, prev_half, curr_half, now, fps);
      portENTER_CRITICAL(&g_state_mux);
      g_flow = f;
      portEXIT_CRITICAL(&g_state_mux);
    }

    uint8_t* tmp = prev;
    prev = curr;
    curr = tmp;
    uint8_t* htmp = prev_half;
    prev_half = curr_half;
    curr_half = htmp;
    have_prev = true;
    last_t = now;

    taskYIELD();
  }
}

void printStatus() {
  FlowState f;
  PoseState p;
  portENTER_CRITICAL(&g_state_mux);
  f = g_flow;
  p = g_pose;
  portEXIT_CRITICAL(&g_state_mux);

  Serial.printf("STATUS,cam=%d,imu=%d,psram=%u,cam_rgb565=%d,cam_fps=%.1f,vision_ms=%.2f,track_res=%dx%d,detected=%u,tracks=%u,sat=%u,flow=%.2f/%.2f,stationary=%d\n",
                (int)g_camera_ok, (int)g_imu_ok, (unsigned)ESP.getFreePsram(),
                (int)g_camera_rgb565, f.fps, f.proc_ms, CAM_W, CAM_H,
                f.detected, f.tracks, f.saturated, f.dx, f.dy, (int)p.stationary);
}

void handleCommand(String s) {
  s.trim();
  s.toUpperCase();
  if (s == "STATUS") {
    printStatus();
  } else if (s == "RESET_POSE") {
    resetPose();
  } else if (s == "CSV 1") {
    g_csv = true;
  } else if (s == "CSV 0") {
    g_csv = false;
  } else if (s.length()) {
    Serial.println("ERR,commands=STATUS|RESET_POSE|CSV 1|CSV 0");
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println();
  Serial.printf("AtomS3R-M12 VIO %s\n", FW_VERSION);

  g_camera_ok = initCamera();
  Serial.printf("[CAM] %s\n", g_camera_ok ? "OK" : "FAILED");

  g_imu_ok = initImu();
  Serial.printf("[IMU] %s\n", g_imu_ok ? "OK" : "FAILED");

  if (g_imu_ok) {
    calibrateGyro();
  }

  g_gray_a = (uint8_t*)ps_malloc((size_t)CAM_W * CAM_H);
  g_gray_b = (uint8_t*)ps_malloc((size_t)CAM_W * CAM_H);
  g_half_a = (uint8_t*)ps_malloc((size_t)PYR_W * PYR_H);
  g_half_b = (uint8_t*)ps_malloc((size_t)PYR_W * PYR_H);
  if (!g_gray_a || !g_gray_b || !g_half_a || !g_half_b) {
    Serial.println("[CAM] tracking buffers allocation failed");
    g_camera_ok = false;
  }

  if (g_imu_ok) {
    xTaskCreatePinnedToCore(imuTask, "imu", 6144, nullptr, 5, nullptr, 1);
  }
  if (g_camera_ok) {
    xTaskCreatePinnedToCore(cameraTask, "camera", 8192, nullptr, 2, nullptr, 0);
  }

  Serial.println("READY");
  Serial.println("POSE,t_us,px,py,pz,vx,vy,vz,qw,qx,qy,qz,roll_deg,pitch_deg,yaw_deg,flow_x,flow_y,detected,tracks,sat,flow_valid,cam_fps,vision_ms,stationary");
  printStatus();
}

void loop() {
  static uint32_t last_print = 0;
  static String cmd;

  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.length()) {
        handleCommand(cmd);
        cmd = "";
      }
    } else if (cmd.length() < 80) {
      cmd += c;
    }
  }

  const uint32_t now_ms = millis();
  if (g_csv && now_ms - last_print >= TELEMETRY_PERIOD_MS) {
    last_print = now_ms;

    FlowState f;
    PoseState p;
    portENTER_CRITICAL(&g_state_mux);
    f = g_flow;
    p = g_pose;
    portEXIT_CRITICAL(&g_state_mux);

    Serial.printf("POSE,%llu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.7f,%.7f,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%d,%.2f,%.2f,%d\n",
                  (unsigned long long)p.t_us,
                  p.p.x,p.p.y,p.p.z,
                  p.v.x,p.v.y,p.v.z,
                  p.q.w,p.q.x,p.q.y,p.q.z,
                  p.roll,p.pitch,p.yaw,
                  f.dx,f.dy,f.detected,f.tracks,f.saturated,(int)f.valid,f.fps,f.proc_ms,(int)p.stationary);
  }

  delay(2);
}
