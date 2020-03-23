#include "geometry/translation2d.hpp"
#include "main.h"

Translation2d::Translation2d(QLength nx, QLength ny)
    : m_x(nx), m_y(ny) {}

QLength Translation2d::distance(const Translation2d& other) const {
    return std::hypot(other.m_x.convert(meter) - m_x.convert(meter), 
                      other.m_y.convert(meter) - m_y.convert(meter)) * meter;
}

QLength Translation2d::norm() const {
    return std::hypot(m_x.convert(meter), m_y.convert(meter)) * meter;
}

Translation2d Translation2d::rotateBy(const Rotation2d& other) const {
    return {m_x * other.cos() - m_y * other.sin(),
            m_x * other.sin() + m_y * other.cos()};
}

Translation2d Translation2d::operator+(const Translation2d& other) const {
    return {x() + other.x(), y() + other.y()};
}

Translation2d& Translation2d::operator+=(const Translation2d& other) {
    m_x += other.m_x;
    m_y += other.m_y;
    return *this;
}

Translation2d Translation2d::operator-(const Translation2d& other) const {
    return *this + -other;
}

Translation2d& Translation2d::operator-=(const Translation2d& other) {
    *this += -other;
    return *this;
}

Translation2d Translation2d::operator-() const { return {-1 * m_x, -1 * m_y}; }

Translation2d Translation2d::operator*(double scalar) const {
    return {scalar * m_x, scalar * m_y};
}

Translation2d& Translation2d::operator*=(double scalar) {
    m_x *= scalar;
    m_y *= scalar;
    return *this;
}

Translation2d Translation2d::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

bool Translation2d::operator==(const Translation2d& other) const {
    return (m_x - other.m_x).abs() < 1e-9_m &&
           (m_y - other.m_y).abs() < 1e-9_m;
}

bool Translation2d::operator!=(const Translation2d& other) const {
     return !operator==(other);
}

Translation2d& Translation2d::operator/=(double scalar) {
    *this *= (1.0 / scalar);
    return *this;
}
