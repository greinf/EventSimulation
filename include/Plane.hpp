#include <Eigen/Dense>

class Plane : public Object {
public:
    Plane(const Eigen::Vector3d& center,
        const Eigen::Vector3d& u,
        const Eigen::Vector3d& v,
        double width,
        double height)
        : m_center(center), m_u(u.normalized()), m_v(v.normalized()),
        m_width(width), m_height(height)
    {
        m_normal = m_u.cross(m_v).normalized(); //Cross product between the directions Vectors for the plane. 
    }

    void translate(const Eigen::Vector3d& offset) {
        m_center += offset;
    }

    void rotate(const Eigen::Matrix3d& rotation) {
        m_u = rotation * m_u;
        m_v = rotation * m_v;
        m_normal = m_u.cross(m_v).normalized();
    }

    bool intersect(const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_dir,
        Eigen::Vector3d& hit_point,
        Eigen::Vector3d& surface_normal,
        double& distance) override;
    
    virtual void transform(std::int64_t timestamp, Scene::translation movement_mode) override {}; //not created for now


    //calculate_brignthness and check_brightness_changes are dummies to check just functionality for now. 
    float calculate_brightness(const Eigen::Vector3d& hit_point, const Eigen::Vector3d& normal) const override {
        Eigen::Vector3d light_dir(0, 0, -1); // light comes from camera direction
        return std::max(0.0f, static_cast<float>(normal.dot(-light_dir.normalized())));
    }

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


private:
    Eigen::Vector3d m_center;
    Eigen::Vector3d m_u, m_v;  // Tangents
    Eigen::Vector3d m_normal;  // Cross product of u and v
    double m_width, m_height;
};
