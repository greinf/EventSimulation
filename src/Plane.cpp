#include "Plane.hpp"
#include <Eigen/Dense>

bool Plane::intersect(const Eigen::Vector3d& ray_origin,
    const Eigen::Vector3d& ray_dir,
    Eigen::Vector3d& hit_point,
    Eigen::Vector3d& surface_normal,
    double& distance) override{

    double denom = m_normal.dot(ray_dir);
    if (std::abs(denom) < 1e-6) {
        return false; // Ray is parallel to the plane
    }

    Eigen::Vector3d diff = m_center - ray_origin; //Difference between center points
    distance = diff.dot(m_normal) / denom; // (ray_origin + t*ray_direction-m_center)*m_normal = 0 MUST, Therfore search for t = distance.

    if (distance < 0) {
        return false; // Intersection behind the ray origin
    }

    hit_point = ray_origin + distance * ray_dir;

    // Check bounds in the local plane coordinate system
    Eigen::Vector3d rel = hit_point - m_center;
    double u_coord = rel.dot(m_u);
    double v_coord = rel.dot(m_v);

    if (std::abs(u_coord) > m_width / 2.0 || std::abs(v_coord) > m_height / 2.0) {
        return false; // Outside finite plane bounds
    }

    surface_normal = m_normal;
    return true;
    
}