#pragma once
#include <Arduino.h>
#include <math.h>

struct Quatf {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct Vec3f {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

class MahonyImu {
 public:
  void setGains(float kp, float ki) {
    twoKp_ = 2.0f * kp;
    twoKi_ = 2.0f * ki;
  }

  void setFromAccel(float ax, float ay, float az) {
    const float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-6f) return;
    ax /= n; ay /= n; az /= n;

    const float roll  = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    const float yaw = 0.0f;

    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q_.w = cr*cp*cy + sr*sp*sy;
    q_.x = sr*cp*cy - cr*sp*sy;
    q_.y = cr*sp*cy + sr*cp*sy;
    q_.z = cr*cp*sy - sr*sp*cy;
    normalizeQ();
  }

  void update(float gx, float gy, float gz,
              float ax, float ay, float az,
              float dt) {
    if (!(dt > 0.0f) || dt > 0.05f) return;

    const float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an > 1e-6f) {
      ax /= an; ay /= an; az /= an;

      const float halfvx = q_.x*q_.z - q_.w*q_.y;
      const float halfvy = q_.w*q_.x + q_.y*q_.z;
      const float halfvz = q_.w*q_.w - 0.5f + q_.z*q_.z;

      const float halfex = ay*halfvz - az*halfvy;
      const float halfey = az*halfvx - ax*halfvz;
      const float halfez = ax*halfvy - ay*halfvx;

      if (twoKi_ > 0.0f) {
        integral_.x += twoKi_ * halfex * dt;
        integral_.y += twoKi_ * halfey * dt;
        integral_.z += twoKi_ * halfez * dt;
        gx += integral_.x;
        gy += integral_.y;
        gz += integral_.z;
      } else {
        integral_ = {};
      }

      gx += twoKp_ * halfex;
      gy += twoKp_ * halfey;
      gz += twoKp_ * halfez;
    }

    const float halfdt = 0.5f * dt;
    gx *= halfdt; gy *= halfdt; gz *= halfdt;

    const float qa = q_.w;
    const float qb = q_.x;
    const float qc = q_.y;

    q_.w += (-qb*gx - qc*gy - q_.z*gz);
    q_.x += ( qa*gx + qc*gz - q_.z*gy);
    q_.y += ( qa*gy - qb*gz + q_.z*gx);
    q_.z += ( qa*gz + qb*gy - qc*gx);
    normalizeQ();
  }

  Quatf q() const { return q_; }

  Vec3f rotateBodyToWorld(const Vec3f& v) const {
    // q * [0,v] * conj(q)
    const float qw=q_.w, qx=q_.x, qy=q_.y, qz=q_.z;
    Vec3f r;
    const float tx = 2.0f * (qy*v.z - qz*v.y);
    const float ty = 2.0f * (qz*v.x - qx*v.z);
    const float tz = 2.0f * (qx*v.y - qy*v.x);
    r.x = v.x + qw*tx + (qy*tz - qz*ty);
    r.y = v.y + qw*ty + (qz*tx - qx*tz);
    r.z = v.z + qw*tz + (qx*ty - qy*tx);
    return r;
  }

  void eulerDeg(float& roll, float& pitch, float& yaw) const {
    const float sinr = 2.0f * (q_.w*q_.x + q_.y*q_.z);
    const float cosr = 1.0f - 2.0f * (q_.x*q_.x + q_.y*q_.y);
    roll = atan2f(sinr, cosr);

    const float sinp = 2.0f * (q_.w*q_.y - q_.z*q_.x);
    pitch = (fabsf(sinp) >= 1.0f) ? copysignf((float)M_PI / 2.0f, sinp) : asinf(sinp);

    const float siny = 2.0f * (q_.w*q_.z + q_.x*q_.y);
    const float cosy = 1.0f - 2.0f * (q_.y*q_.y + q_.z*q_.z);
    yaw = atan2f(siny, cosy);

    constexpr float k = 180.0f / (float)M_PI;
    roll *= k; pitch *= k; yaw *= k;
  }

 private:
  void normalizeQ() {
    const float n = sqrtf(q_.w*q_.w + q_.x*q_.x + q_.y*q_.y + q_.z*q_.z);
    if (n < 1e-9f) { q_ = {}; return; }
    const float inv = 1.0f / n;
    q_.w*=inv; q_.x*=inv; q_.y*=inv; q_.z*=inv;
  }

  Quatf q_{};
  Vec3f integral_{};
  float twoKp_ = 1.6f;
  float twoKi_ = 0.04f;
};
