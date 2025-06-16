#pragma once

#include "_Camera.hpp"
#include <Eigen/Dense>
#include <cstdint>
#include <optional>
#include <dv-processing/core/core.hpp>
#include <dv-processing/core/utils.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>

template<std::int16_t pixel_x, std::int16_t pixel_y>
class Camera : public _Camera {
public:
    constexpr Camera()
        : m_intrinsic((Eigen::Matrix3d() << m_f_x, 0, m_c_x,
            0, m_f_y, m_c_y,
            0, 0, 1).finished()),
        m_inv_intrinsic(m_intrinsic.inverse()),
        m_extrinsic(Eigen::Matrix4d::Identity()),
        m_extrinsic_inv(m_extrinsic.inverse())
    {
        //std::cout << "problems after that?" << '\n';
        m_rays.resize(m_pixel_y, m_pixel_x);
        calculate_rays();
        //std::cout << "problems before that? ";
    }

    constexpr void calculate_rays();

    constexpr void add_extrinsic_rot();

    constexpr Eigen::Matrix<Eigen::Vector3d, pixel_x, pixel_y>& get_rays() {
        return m_rays;
    }

    // override from _Camera
    Eigen::Vector3d get_camera_pos() const override {
        return m_extrinsic.block<3, 1>(0, 3);
    }

    const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& get_rays_dynamic() const override {
        if constexpr (pixel_x == Eigen::Dynamic || pixel_y == Eigen::Dynamic) {
            return m_rays;
        }
        else {
            static Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> rays_dynamic;
            rays_dynamic.resize(m_pixel_y, m_pixel_x);
            for (int y = 0; y < m_pixel_y; ++y)
                for (int x = 0; x < m_pixel_x; ++x)
                    rays_dynamic(y, x) = m_rays(y, x);
            return rays_dynamic;
        }
    }


    std::optional<bool> check_brightness_change(int x, int y, std::int16_t new_brightness) override {
        std::int16_t old = m_prev_brightness[x][y];
        std::int16_t diff = new_brightness - old;
        //std::cout << "Camera addr: " << this << " | old: " << old << ", new: " << new_brightness << '\n';
        /*
        if (diff < 0) {
            std::cout << "Diff smaller 0 zero deteceted. ";
        }
        if (diff > 0)
            std::cout << "diff bigger 0 detected. ";
            */
        //std::cout << "Brightness value " << new_brightness << '\n';
        

        if (std::abs(diff) > 255*0.2) { //20% der gesamten Range
            bool polarity = diff > 0;  // true = ON, false = OFF
            m_prev_brightness[x][y] = new_brightness;      // nur aktualisieren, wenn Event erkannt wurde!
            return polarity;
        }
        
        return std::nullopt;
    }

    constexpr std::optional<cv::Size> getEventResolution() override {
        return cv::Size(pixel_x, pixel_y);
    }

    dv::EventStore* getEventStore() override {return &m_event_store;}

    std::vector<cv::Mat>* getFrameStore() override { return &m_frame_store; }

private:
    static constexpr std::int16_t m_pixel_x{ pixel_x };
    static constexpr std::int16_t m_pixel_y{ pixel_y };
    static constexpr double m_pixel_pitch{ 0.0185 };
    static constexpr double m_f_x = 486, m_f_y = 486;
    static constexpr double m_c_x = 0, m_c_y = 0;

    Eigen::Matrix3d m_intrinsic{};
    Eigen::Matrix3d m_inv_intrinsic{};
    Eigen::Matrix4d m_extrinsic = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d m_extrinsic_inv{};
    Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> m_rays{};

    std::array<std::array<std::int16_t, pixel_y>, pixel_x> m_prev_brightness{};
    dv::EventStore m_event_store{};
    std::vector<cv::Mat> m_frame_store{};

};


template<std::int16_t pixel_x, std::int16_t pixel_y>
constexpr void Camera<pixel_x, pixel_y>::calculate_rays() {
    
    for (int height = 0; height < m_pixel_y; ++height) {
        for (int width = 0; width < m_pixel_x; ++width) {
            double x_real = (width - (m_pixel_x / 2)) * m_pixel_pitch;
            double y_real = (height - (m_pixel_y / 2)) * m_pixel_pitch;
            //std::cout << "Height " << height << '\n';
            //std::cout << "Width " << width << '\n';
            Eigen::Vector3d pixel_vector(x_real, y_real, 1.0);
            Eigen::Vector3d ray = (m_inv_intrinsic * pixel_vector);

            m_rays(height, width) = ray;
        }
    }
    //std::cout << "Stop here? ";
    if (!m_extrinsic_inv.isIdentity()) {
        add_extrinsic_rot();
    }
}

template<std::int16_t pixel_x, std::int16_t pixel_y>
constexpr void Camera<pixel_x, pixel_y>::add_extrinsic_rot() {
    m_rays = m_rays.unaryExpr([this](const Eigen::Vector3d& vec) {
        Eigen::Vector3d result = m_extrinsic_inv.block<3, 3>(0, 0) * vec;
        result = result.normalized();
        return result;
        });
}
