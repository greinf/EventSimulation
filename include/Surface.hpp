#include <Eigen/Dense>
#include <numbers>

class Surface {
private:
    Eigen::Vector3d m_vectorU{ 0, 1, 0 }; //Vector in y directon
    Eigen::Vector3d m_vectorV{ 0, 0, 1 }; //Vector in z direction
    Eigen::Vector3d m_origin{ 0, 0, 0 };
    static constexpr double m_frequency{ 0.1 };
    static constexpr double m_phase{ 0 };

    //Matrix<Vector2d, Dynamic, Dynamic> hits;
public:
    explicit Surface(const Eigen::Vector3d& origin)
        : m_origin{ origin }
        {}
    Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic>
        intersection(const Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic>&, const Eigen::Vector3d);
    double gray_value(const Eigen::Vector2d& uv, double frequency, double phase) const;
};
