#include <vector>
#include <iostream>
#include <array>
#include <string>
#include <Eigen/Dense>
#include <cassert>
#include <numbers>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace Eigen;

class Surface {
private:
    Vector3d m_vectorU{ 6, 1, 0 }; //Vector in y directon
    Vector3d m_vectorV{ 0, 0, 1 }; //Vector in z direction
    Vector3d m_origin{ 0, 0, 0 };
    double m_frequency{ 0.1 };
    double m_phase{ 0 };

    //Matrix<Vector2d, Dynamic, Dynamic> hits;
public:
    Surface(Vector3d origin)
        : m_origin{ origin }
    {}
    Matrix<Vector2d, Dynamic, Dynamic> intersection(const Matrix<Vector3d, Dynamic, Dynamic>&,const Vector3d); 
    double gray_value(const Vector2d& uv, double frequency, double phase);
};

double Surface::gray_value(const Vector2d& uv, double m_frequency, double m_phase)  {
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

Matrix<Vector2d, Dynamic, Dynamic> Surface::intersection(const Matrix<Vector3d, Dynamic, Dynamic>& rays, const Vector3d ray_origin) {
    Matrix<Vector2d, Dynamic, Dynamic> hits(rays.rows(), rays.cols());

    for (int i = 0; i < rays.rows(); ++i) {
        for (int j = 0; j < rays.cols(); ++j) {
            Vector3d ray = rays(i, j);

            Matrix3d gls{};
            gls.col(0) = ray;
            gls.col(1) = m_vectorU.normalized();
            gls.col(2) = m_vectorV.normalized();

            Vector3d lsg = ray_origin - m_origin;

            //Build GLS and solve for lsb-vector lsg
            Vector3d tuv = gls.fullPivLu().solve(lsg);
            
            if (abs(tuv.cwiseAbs().maxCoeff()) > 10000 || tuv[0] <= 0) { //just asumed value
                hits(i, j) = Vector2d::Zero();
            }
            else {
                hits(i, j) = tuv.segment<2>(1); 
            }
        }
    }
    return hits;
}

class Objects {
private:
    std::array<double, 3> m_position{ 0, 0, 0 };
public:
    Objects() = default;
};

class Camera {
private:
    const int m_pixel_x{ 240 };
    const int m_pixel_y{ 180 };
    double m_pixel_pitch{ 0.0185 }; //18.5 * 10 ^-6m pixelpitch
    double m_f_x = 486, m_f_y = 486;
    double m_c_x = 0, m_c_y = 0;
    Matrix3d m_intrinsic{};
    Matrix3d m_inv_intrinsic{};
    Matrix4d m_extrinsic = Matrix4d::Identity();
    Matrix4d m_extrinsic_inv{};
    Matrix<Vector3d, Dynamic, Dynamic> m_rays{};
    //Vector3d m_camera_pos{ 10, 0, 0 };

public:
    Camera()
        : m_intrinsic((Matrix3d() << m_f_x, 0, m_c_x,
            0, m_f_y, m_c_y,
            0, 0, 1).finished()),
        m_extrinsic((Matrix4d() << 0, 0, -1, 3000,
            0, 1, 0, 0,
            1, 0, 0, 0,
            0, 0, 0, 1).finished()),

        m_inv_intrinsic(m_intrinsic.inverse()),
        m_extrinsic_inv(m_extrinsic.inverse()),
        m_rays{calculate_rays()}
    {}

    Matrix<Vector3d, Dynamic, Dynamic> calculate_rays();
    void add_translation(Matrix<Vector3d, Dynamic, Dynamic>&);
    void add_extrinsic(Matrix<Vector3d, Dynamic, Dynamic>&);
    const Matrix<Vector3d, Dynamic, Dynamic>& get_rays() { return m_rays; };
    const Vector3d get_camera_pos(){ return m_extrinsic.block<3,1>(0,3); };

};

Matrix<Vector3d, Dynamic, Dynamic> Camera::calculate_rays() {
    Matrix<Vector3d, Dynamic, Dynamic> rays(m_pixel_y, m_pixel_x);

    for (int height = 0; height < m_pixel_y; ++height) { 
        for (int width = 0; width < m_pixel_x; ++width) {
            double x_real = (width - (m_pixel_x/2)) * m_pixel_pitch;
            double y_real = (height - (m_pixel_y/2)) * m_pixel_pitch;
            // Form the pixel vector in the image plane
            Vector3d pixel_vector(x_real, y_real, 1.0);
            // Transform the pixel vector to get the ray
            Vector3d ray = (m_inv_intrinsic * pixel_vector);
            //ray = ray.normalized();
            rays(height, width) = ray;
        }
    }
    if (m_extrinsic_inv.isIdentity()) {
        return rays;
    }
    /*
    else if (m_extrinsic_inv.topLeftCorner(3, 3).isIdentity()) {
        add_translation(rays);
    }
    */
    else {
        add_extrinsic(rays);
    }
    return rays;
}

void Camera::add_extrinsic(Matrix<Vector3d, Dynamic, Dynamic>& rays) {
    rays = rays.unaryExpr([this](const Vector3d& vec) {
        Vector3d result = m_extrinsic_inv.block<3, 3>(0, 0) * vec;
        result = result.normalized();
        return result;
        });
}

/*
//FUN parts different implmentation same result
//Calculate with normal lambda fucntion to get the same result
rays = rays.unaryExpr([translation](Vector3d vec) -> Vector3d { //extracts all elemnts vec from the matrix
    return vec + translation;
    });

//Calculate with loops
for (int height{ 0 }; height < m_pixel_y; ++height) {
    for (int width{ 0 }; width < m_pixel_x; ++width) {
        rays(height,width) += translation;
    }
*/

struct AddTranslationFunctor {
    Vector3d m_translation;
    AddTranslationFunctor(const Vector3d& translation)
        : m_translation(translation) {}
    typedef Vector3d result_type;
    Vector3d operator()(const Vector3d& vec) const {
        return vec + m_translation;
    }
};

void Camera::add_translation(Matrix<Vector3d, Dynamic, Dynamic>& rays) {
    Vector3d translation = m_extrinsic_inv.block<3, 1>(0, 3);
    AddTranslationFunctor addTranslation(translation);
    rays = rays.unaryExpr(addTranslation);
}

class Scene {
private:
    std::vector<Camera> m_camera{};
    std::vector<Surface> m_surface{};
    std::vector<Objects> m_objects{};
public:
    Scene() = default;
    template <typename T>
    void object_input(const T& obj);
};

template <typename T>
void Scene::object_input(const T& obj) {
    if constexpr (std::is_same<T, Camera>::value) {
        m_camera.push_back(obj);
        std::cout << "Camera added to the scene.\n";
    }
    else if constexpr (std::is_same<T, Surface>::value) {
        m_surface.push_back(obj);
        std::cout << "Surface added to the scene.\n";
    }
    else if constexpr (std::is_same<T, Objects>::value) {
        m_objects.push_back(obj);
        std::cout << "Object added to the scene.\n";
    }
    else {
        std::cerr << "Datatype not found!\n";
        std::exit(0);
    }
}

cv::Mat eigenToCv(const Eigen::MatrixXd& eigenMat) {
    // Create an OpenCV matrix with the same dimensions as the Eigen matrix
    cv::Mat image(eigenMat.rows(), eigenMat.cols(), CV_8UC1); // 8-bit single-channel image

    // Scale the values from Eigen::MatrixXd (assuming 0-255 range)
    for (int y = 0; y < eigenMat.rows(); ++y) {
        for (int x = 0; x < eigenMat.cols(); ++x) {
            // Clamp the value to [0, 255] and cast to uchar
            image.at<uchar>(y, x) = static_cast<uchar>(std::clamp(eigenMat(y, x), 0.0, 255.0));
        }
    }
    return image;
}

template<typename Matrix>
void printMatrixSubset(const Matrix& mat, int maxRows = 10, int maxCols = 10, const std::string& name = "") {
    if (!name.empty()) {
        std::cout << name << " (showing " << maxRows << "x" << maxCols << "):\n";
    }
    for (int i = 0; i < maxRows; ++i) {
        for (int j = 0; j < maxCols; ++j) {
            std::cout << mat(i, j) << " ";
        }
        if (mat.cols() > maxCols) {
            std::cout << "...";
        }
        std::cout << "\n";
    }
    if (mat.rows() > maxRows) {
        std::cout << "...\n";
    }
    std::cout << "\n";
}

int main() {
    Surface plain({ 0, 0, 0 });
    Camera camera1;
    // Get rays from the camera

    // Get rays from the camera
    Matrix<Vector3d, Dynamic, Dynamic> rays = camera1.get_rays();
    Vector3d ray_origin = camera1.get_camera_pos(); // Camera origin

    // Compute intersections
    Matrix<Vector2d, Dynamic, Dynamic> rays_hit_point_surf = plain.intersection(rays, ray_origin);
    //std::cout << rays_hit_point_surf(90 ,120);

    // Create a 240x180 image for gray values
    MatrixXd gray_image(180, 240); // Rows = height, Cols = width

    // Compute gray values for each pixel
    for (int y = 0; y < gray_image.rows(); ++y) {
        for (int x = 0; x < gray_image.cols(); ++x) {
            Vector2d uv = rays_hit_point_surf(y, x); // (u, v) intersection point
            if (uv == Vector2d::Zero()) {
                // No intersection, set to black (0 intensity)
                gray_image(y, x) = 0;
            }
            else {
                // Compute gray value
                gray_image(y, x) = plain.gray_value(uv, 1, 0.0); 
            }
        }
    }
    //printMatrixSubset(gray_image);
    cv::Mat img = eigenToCv(gray_image);
    std::cout << "Attempting to show image\n";
    cv::imshow("image", img);
    cv::imwrite("RayCasting.png", img);
    cv::waitKey(0);
    char c{};
    std::cin >> c;
    
    return 0;
}

