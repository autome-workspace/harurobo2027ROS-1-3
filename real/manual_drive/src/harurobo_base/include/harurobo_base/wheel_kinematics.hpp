#ifndef HARUROBO_BASE__WHEEL_KINEMATICS_HPP_
#define HARUROBO_BASE__WHEEL_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace harurobo_base
{
struct WheelGeometry
{
  double x_m, y_m;
  double tx, ty;
  double radius_m;
  double max_rad_s;
};

// Wheel order is supplied by the caller. Motor polarity/gearing belong at CAN boundary.
class WheelKinematics
{
public:
  explicit WheelKinematics(const std::array<WheelGeometry, 4> & wheels)
  {
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      const auto & w = wheels[i];
      for (double v : {w.x_m, w.y_m, w.tx, w.ty, w.radius_m, w.max_rad_s}) {
        if (!std::isfinite(v)) {
          throw std::invalid_argument("nonfinite wheel geometry");
        }
      }
      if (w.radius_m <= 0 || w.max_rad_s <= 0 ||
        std::abs(std::hypot(w.tx, w.ty) - 1.0) > 1e-6)
      {
        throw std::invalid_argument("wheel radius/limit must be positive; drive vector must be unit");
      }
      rows_[i] = {w.tx / w.radius_m, w.ty / w.radius_m,
        (-w.tx * w.y_m + w.ty * w.x_m) / w.radius_m};
      limits_[i] = w.max_rad_s;
      for (double v : rows_[i]) {
        if (!std::isfinite(v)) {
          throw std::invalid_argument("wheel geometry overflows kinematics");
        }
      }
    }
    // Normalized Gram-Schmidt detects rank < 3, independently of column units.
    std::array<std::array<double, 4>, 3> basis{};
    for (std::size_t col = 0; col < 3; ++col) {
      auto & vector = basis[col];
      double norm = 0;
      for (std::size_t i = 0; i < 4; ++i) {
        vector[i] = rows_[i][col];
        norm = std::hypot(norm, vector[i]);
      }
      if (norm == 0 || !std::isfinite(norm)) {
        throw std::invalid_argument("wheel matrix is singular or nonfinite");
      }
      for (auto & v : vector) {v /= norm;}
      for (std::size_t previous = 0; previous < col; ++previous) {
        double dot = 0;
        for (std::size_t i = 0; i < 4; ++i) {dot += vector[i] * basis[previous][i];}
        for (std::size_t i = 0; i < 4; ++i) {vector[i] -= dot * basis[previous][i];}
      }
      norm = 0;
      for (double v : vector) {norm = std::hypot(norm, v);}
      if (norm < 1e-6) {
        throw std::invalid_argument("wheel matrix must have rank 3");
      }
      for (auto & v : vector) {v /= norm;}
    }
  }

  std::array<double, 4> solve(double vx, double vy, double wz) const
  {
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
      throw std::invalid_argument("nonfinite body velocity");
    }
    std::array<double, 4> result{};
    double scale = 1;
    for (std::size_t i = 0; i < 4; ++i) {
      result[i] = rows_[i][0] * vx + rows_[i][1] * vy + rows_[i][2] * wz;
      if (!std::isfinite(result[i])) {
        throw std::invalid_argument("wheel speed overflow");
      }
      if (std::abs(result[i]) > limits_[i]) {
        scale = std::min(scale, limits_[i] / std::abs(result[i]));
      }
    }
    for (auto & value : result) {value *= scale;}
    return result;
  }

private:
  std::array<std::array<double, 3>, 4> rows_{};
  std::array<double, 4> limits_{};
};
}  // namespace harurobo_base

#endif  // HARUROBO_BASE__WHEEL_KINEMATICS_HPP_
