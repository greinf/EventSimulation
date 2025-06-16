#include <vector>
#include <iostream>
#include <array>
#include <string>
#include <Eigen/Dense>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "Camera.hpp"
#include "Scene.hpp"
#include "Object3D.hpp"
/*
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

*/

int main() {
    Scene<3> szene(1,1); //in 3 Dimensional space 1 camera and 1 object (Surface in this case)
    szene.addcamera(std::make_unique<Camera<240, 180>>());
    szene.addobject(std::make_unique<Plane>());
    const Scene_constants::translation translation{ Scene_constants::translation::translation_left };
    const Scene_constants::velocity velocity{ Scene_constants::velocity::fast };
    const Scene_constants::camera_para storage_mode{ Scene_constants::camera_para::store_frameAndEvents };
    szene.start_simulation(translation, velocity, storage_mode);
    std::cout << "finished simulation ";
    szene.visualizeEvents();
    szene.visualizeFrames();








    /*
    Surface plain(Eigen::Vector3d{ 0, 0, 0 });
    Camera camera1;
    // Get rays from the camera

    // Get rays from the camera
    Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> rays = camera1.get_rays();
    Eigen::Vector3d ray_origin = camera1.get_camera_pos(); // Camera origin

    // Compute intersections
    Eigen::Matrix<Eigen::Vector2d, Eigen::Dynamic, Eigen::Dynamic> rays_hit_point_surf = plain.intersection(rays, ray_origin);
    //std::cout << rays_hit_point_surf(90 ,120);

    // Create a 240x180 image for gray values
    Eigen::MatrixXd gray_image(180, 240); // Rows = height, Cols = width

    // Compute gray values for each pixel
    for (int y = 0; y < gray_image.rows(); ++y) {
        for (int x = 0; x < gray_image.cols(); ++x) {
            Eigen::Vector2d uv = rays_hit_point_surf(y, x); // (u, v) intersection point
            if (uv == Eigen::Vector2d::Zero()) {
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
    */
}

