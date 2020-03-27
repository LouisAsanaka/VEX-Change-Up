#include "libraidzero/geometry/rotation2d.hpp"
#include "main.h"
#include <cmath>

Rotation2d::Rotation2d(QAngle value)
    : m_value(value),
      m_cos(std::cos(value.convert(radian))),
      m_sin(std::sin(value.convert(radian))) {}

Rotation2d::Rotation2d(double x, double y) {
    const auto magnitude = std::hypot(x, y);
    if (magnitude > 1e-6) {
        m_sin = y / magnitude;
        m_cos = x / magnitude;
    } else {
        m_sin = 0.0;
        m_cos = 1.0;
    }
    m_value = std::atan2(m_sin, m_cos) * radian;
}

Rotation2d Rotation2d::operator+(const Rotation2d& other) const {
  return rotateBy(other);
}

Rotation2d& Rotation2d::operator+=(const Rotation2d& other) {
  double ncos = cos() * other.cos() - sin() * other.sin();
  double nsin = cos() * other.sin() + sin() * other.cos();
  m_cos = ncos;
  m_sin = nsin;
  m_value = std::atan2(m_sin, m_cos) * radian;
  return *this;
}

Rotation2d Rotation2d::operator-(const Rotation2d& other) const {
  return *this + -other;
}

Rotation2d& Rotation2d::operator-=(const Rotation2d& other) {
  *this += -other;
  return *this;
}

Rotation2d Rotation2d::operator-() const { return Rotation2d(m_value * -1); }

Rotation2d Rotation2d::operator*(double scalar) const {
  return Rotation2d(m_value * scalar);
}

bool Rotation2d::operator==(const Rotation2d& other) const {
  return (m_value - other.m_value).abs() < 1E-9_rad;
}

bool Rotation2d::operator!=(const Rotation2d& other) const {
  return !operator==(other);
}

Rotation2d Rotation2d::rotateBy(const Rotation2d& other) const {
  return {cos() * other.cos() - sin() * other.sin(),
          cos() * other.sin() + sin() * other.cos()};
}

