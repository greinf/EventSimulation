#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Object3D.hpp"
#include "Scene.hpp"

class Plane : public Object3D {
public:
    //All distances in mm with all default values. 
    Plane(const Eigen::Vector3d& center = Eigen::Vector3d (-600,0,100000), //camera looks into z direction form origin. starting point 
        //one meter from the origin in z directoin and translated -500 with respect to x
        const Eigen::Vector3d& u = Eigen::Vector3d (1,0,0),
        const Eigen::Vector3d& v = Eigen::Vector3d (0,1,0),
        double width = 240,
        double height = 180)
        : m_center(center), m_u(u.normalized()), m_v(v.normalized()),
        m_width(width), m_height(height)
    {
        m_normal = m_u.cross(m_v).normalized(); //Cross product between the directions Vectors for the plane. 
    }

    Eigen::Vector3d getObjPosition() const override {
        return m_center;
    }

    void translate(const Eigen::Vector3d& offset, const std::int64_t time_step) {
        constexpr double microseconds_to_seconds = 1e-6;
        double time_step_seconds = (microseconds_to_seconds * static_cast<double>(time_step))*1000; //everything is calculated in mm
        //std::cout << offset * time_step_seconds;
        m_center += offset * time_step_seconds;
    }

    void rotate(float angle, std::int64_t timestep) {
        angle = static_cast<double>(angle);
        constexpr double microseconds_to_seconds = 1e-6;
        double time_step_seconds = microseconds_to_seconds * static_cast<double>(timestep);
        Eigen::AngleAxisd rotation(angle * time_step_seconds, Eigen::Vector3d::UnitY()); //rotation around the y-Axis with repsect to the timestep_width
        m_u = rotation * m_u;
        m_v = rotation * m_v;
        m_normal = m_u.cross(m_v).normalized();
    }

    bool intersect(const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_dir,
        Eigen::Vector3d& hit_point,
        Eigen::Vector3d& surface_normal,
        double& distance) override;
    
    void transform(const std::int64_t timesteps,
        const Scene_constants::translation movement_mode,
        const Scene_constants::velocity movement_speed) override {

        float velocity_t{};
        float velocity_r{};
        switch (static_cast<int>(movement_speed)) {
        case(static_cast<int>(Scene_constants::velocity::slow)):
            velocity_t = 0.1;  //0.25 m/s
            velocity_r = float(std::numbers::pi / 16);
            break;
        case(static_cast<int>(Scene_constants::velocity::medium)):
            velocity_t = 0.3;  //0.25 m/s
            velocity_r = float(std::numbers::pi / 8);
            break;
        case(static_cast<int>(Scene_constants::velocity::fast)):
            velocity_t = 1;  //0.25 m/s
            velocity_r = float(std::numbers::pi / 2);
            break;
        }

        switch (static_cast<int>(movement_mode)) {
        case(static_cast<int>(Scene_constants::translation::frame_static)):
            return;
        case(static_cast<int>(Scene_constants::translation::translation_left)): {  //When declaring new variables in a switch case
                //they must be in a seprate scop therefore are getting destroyed after leaving the scope again. 
            const Eigen::Vector3d& translation_vector = Eigen::Vector3d(1, 0, 0) * velocity_t;
            translate(translation_vector, timesteps);
            break;
        }
        case(static_cast<int>(Scene_constants::translation::rotation_clockwise)):
            rotate(velocity_r, timesteps);
            break;
        }
    }
    
    std::int16_t lambertian_basic(const Eigen::Vector3d& hit_point, 
        const Eigen::Vector3d& normal,
        const Eigen::Vector3d& light_position) const override {
        Eigen::Vector3d light_dir = (light_position - hit_point).normalized();  
        Eigen::Vector3d fixed_normal = normal.normalized();  // Should already be normalized
        if (fixed_normal.dot(light_dir) < 0)
            fixed_normal = -fixed_normal;
        std::int16_t brightness = static_cast<std::int16_t>(fixed_normal.dot(light_dir)*255);
        
        //std::cout << brightness;
        return brightness; 
    }

    //calculate_brignthness and check_brightness_changes are dummies to check just functionality for now. 
    std::int16_t calculate_brightness(const Eigen::Vector3d& hit_point,
        const Eigen::Vector3d& normal,
        const Eigen::Vector3d& light_position,
        const Scene_constants::lightning light) const override {
        switch (static_cast<int>(light)) {
        case(static_cast<int>(Scene_constants::lightning::lambertian_basic)):
            return lambertian_basic(hit_point, normal, light_position);
        case(static_cast<int>(Scene_constants::lightning::singel_light_source)):  //to do implement raycsting for singel light source. 
            std::cout << "not Yet implemented" << '\n';
            std::abort();
        case(static_cast<int>(Scene_constants::lightning::max_elements_l)):
            std::cout << "out of bounds. Alarm!!!"; //maybe multiple or more complex light source. 
        }
    }
    /*
    bool check_brightness_change(int x, int y, float new_brightness) override {
        // Bounds check (avoid access violations)
        if (x < 0 || x >= static_cast<int>(m_previous_brightness.size()) ||
            y < 0 || y >= static_cast<int>(m_previous_brightness[0].size())) {
            return false;
        }

        float& old = m_previous_brightness[x][y];
        bool changed = std::abs(new_brightness - old) > 0.1f;  // Sensitivity threshold
        old = new_brightness;
        return changed;
    }
    */

private:
    Eigen::Vector3d m_center;
    Eigen::Vector3d m_u, m_v;  // Tangents
    Eigen::Vector3d m_normal;  // Cross product of u and v
    double m_width, m_height;
};
