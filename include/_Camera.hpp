#pragma once
#include <Eigen/Dense>
#include <optional>
#include <dv-processing/core/core.hpp>


class _Camera {
public:
    virtual ~_Camera() = default;

    virtual Eigen::Vector3d get_camera_pos() const = 0;

    virtual const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& get_rays_dynamic() const = 0;

    virtual std::optional<bool> check_brightness_change(int x, int y, std::int16_t new_brightness) = 0; 

    virtual std::optional<cv::Size> getEventResolution() = 0;

    virtual dv::EventStore* getEventStore() = 0;

    virtual std::vector<cv::Mat>* getFrameStore() = 0;
};
