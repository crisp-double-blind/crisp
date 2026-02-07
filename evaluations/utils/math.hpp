#pragma once

#include <cmath>
#include <numbers>

#include <Eigen/Dense>

namespace utils {
template <typename T>
constexpr T deg2rad(T deg) {
  return deg * std::numbers::pi_v<T> / 180;
}

template <typename T>
constexpr T rad2deg(T rad) {
  return rad * 180 / std::numbers::pi_v<T>;
}

template <typename T>
T sind(T deg) {
  return std::sin(deg2rad(deg));
}

template <typename T>
T cosd(T deg) {
  return std::cos(deg2rad(deg));
}

template <typename T>
T tand(T deg) {
  return std::tan(deg2rad(deg));
}

template <typename T>
Eigen::Quaternion<T> aa2quat(T deg, Eigen::Vector3<T> const& axis) {
  return Eigen::Quaternion<T>(
    Eigen::AngleAxis<T>(deg2rad(deg), axis.normalized()));
}
}  // namespace utils
