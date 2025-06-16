#pragma once
#include "Scene_constants.hpp"
#include <Eigen/Dense>

//Base class for simulation of Events 
class Object3D {
public:
    virtual ~Object3D() = default;

    // Ray intersection: returns true if hit, and fills hit point & normal
    virtual bool intersect(
        const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_direction,
        Eigen::Vector3d& hit_point,
        Eigen::Vector3d& surface_normal,
        double& distance
    ) = 0;

    virtual void transform(const std::int64_t timesteps, 
        const Scene_constants::translation movement_mode, 
        const Scene_constants::velocity movement_speed) = 0;
    virtual Eigen::Vector3d getObjPosition() const = 0;
    virtual std::int16_t calculate_brightness(const Eigen::Vector3d& hit_point, 
        const Eigen::Vector3d& normal, 
        const Eigen::Vector3d& light_position,
        const Scene_constants::lightning lightning) const = 0;
    virtual std::int16_t lambertian_basic(const Eigen::Vector3d& hit_point,
        const Eigen::Vector3d& normal,
        const Eigen::Vector3d& light_position) const = 0;
    //virtual bool check_brightness_change(int x, int y, float new_brightness) = 0;
};
