/**
 * @file
 * @brief PCL point datatype for use with ouster sensors
 */

#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <chrono>
#include <functional>

#include "ouster/lidar_scan.h"

namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint8_t ring;
    std::uint16_t noise;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static inline std::function<
        Point(std::ptrdiff_t, std::ptrdiff_t, std::chrono::nanoseconds,
              std::chrono::nanoseconds, std::uint32_t, std::uint16_t, std::uint16_t, std::uint16_t)>
    get_from_pixel(const ouster::XYZLut& xyz_lut, size_t w, size_t h) {
        return [xyz_lut, w, h](std::ptrdiff_t u, std::ptrdiff_t v,
                               std::chrono::nanoseconds ts,
                               std::chrono::nanoseconds scan_ts, std::uint32_t range,
                               std::uint16_t intensity, std::uint16_t noise,
                               std::uint16_t reflectivity) -> Point {
            const auto xyz = xyz_lut.direction.row(u * w + v) * range +
                             xyz_lut.offset.row(u * w + v);
            return {static_cast<float>(xyz(0)),
                    static_cast<float>(xyz(1)),
                    static_cast<float>(xyz(2)),
                    0.0f,
                    static_cast<float>(intensity),
                    static_cast<std::uint32_t>((ts - scan_ts).count()),
                    static_cast<std::uint16_t>(reflectivity),
                    static_cast<std::uint8_t>(u),
                    static_cast<std::uint16_t>(noise),
                    static_cast<std::uint32_t>(range)};
        };
    }
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, noise, noise)
    (std::uint32_t, range, range)
)
// clang-format on
