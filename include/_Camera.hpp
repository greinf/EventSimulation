#pragma once
#include <Eigen/Dense>

class _Camera {
public:
    virtual ~_Camera() = default;

    virtual Eigen::Vector3d get_camera_pos() const = 0;

    virtual const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& get_rays_dynamic() const = 0;
};
