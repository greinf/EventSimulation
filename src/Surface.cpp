#include "Surface.hpp"
#include <numbers>

double Surface::gray_value(const Eigen::Vector2d& uv, double m_frequency, double m_phase) const{
    //double length = uv.cwiseAbs();
    double a{ 0 };
    if (uv.cwiseAbs().maxCoeff() > 9) { //Frame the picture
        return a;
    }
    else {
        a = abs((255 * (std::sin(2 * std::numbers::pi * m_frequency * uv.norm() + m_phase) + 1)));
        return a;
    }
}

Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic> 
    Surface::intersection(const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>& rays, const Eigen::Vector3d ray_origin) {
    Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic> hits(rays.rows(), rays.cols());

    for (int i = 0; i < rays.rows(); ++i) {
        for (int j = 0; j < rays.cols(); ++j) {
            Eigen::Vector3d ray = rays(i, j);

            Eigen::Matrix3d gls{};
            gls.col(0) = ray;
            gls.col(1) = m_vectorU.normalized();
            gls.col(2) = m_vectorV.normalized();

            Eigen::Vector3d lsg = ray_origin - m_origin;

            //Build GLS and solve for lsb-vector lsg
            Eigen::Vector3d tuv = gls.fullPivLu().solve(lsg);

            if (abs(tuv.cwiseAbs().maxCoeff()) > 10000 || tuv[0] <= 0) { //just asumed value
                hits(i, j) = Eigen::Vector2d::Zero();
            }
            else {
                hits(i, j) = tuv.segment<2>(1);
            }
        }
    }
    return hits;
}