#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <dv-processing/core/core.hpp>
#include <dv-processing/core/utils.hpp>
#include <dv-processing/visualization/event_visualizer.hpp>
#include <Object3D.hpp>
#include <_Camera.hpp>
#include <Camera.hpp>
#include <Surface.hpp>
#include <Plane.hpp>
#include <vector>
#include <memory>
#include <optional>
#include "Scene_constants.hpp"
#include <opencv2/highgui.hpp>

//For now just save use with one camera. 
template<std::int8_t Dimensions>
class Scene {  //WIDTH and HEIGHT no compile time constants. 
public:
	Scene(std::size_t n_cameras, std::size_t n_objects) :
		timing{ 0 }
	{
		//cameras.reserve(n_cameras);
		//objects.reserve(n_objects);
		/*
		for (std::size_t i = 0; i < n_cameras; ++i) {
			cameras.emplace_back(std::make_unique<Camera<Dimensions, WIDTH, HEIGHT>>());
		}
		for (std::size_t u = 0; u < n_objects; ++u) {
			surfaces.emplace_back(std::make_unique<Surface<WIDTH, HEIGHT>>());
		}
		*/
	}

	void addobject(std::unique_ptr<Object3D> obj) {
		if (!obj) {
			std::cerr << "Warning: Tried to add null object to scene!" << std::endl;
			return;
		}
		objects.emplace_back(std::move(obj));
	}


	template<std::int16_t WIDTH, std::int16_t HEIGHT>
	void addcamera(std::unique_ptr<Camera<WIDTH, HEIGHT>> camera) {
		if (!camera) {
			std::cerr << "Warning: Tried to add null object to scene!" << std::endl;
			return;
		}
		cameras.emplace_back(std::move(camera));
	}

	std::int64_t timing{};
	void start_simulation(Scene_constants::translation t_mode, Scene_constants::velocity v_mode,
		Scene_constants::camera_para storage_mode); //for starting of timer and propably moving the object 
	
	void visualizeEvents() const;
	void visualizeFrames() const;


private:
	static constexpr std::int8_t width = Dimensions;
	std::vector < std::unique_ptr<_Camera>> cameras{};
	std::vector < std::unique_ptr<Object3D>> objects{};
	//dv::EventStore event_store;

};

/*cameras is std::vector<std::unique_ptr<camera<...>> this could be more than camera. In this method only the visulisation 
for the first camera in the container is implemented.*/
template<std::int8_t Dimensions>
void Scene<Dimensions>::visualizeEvents() const {
	if (cameras.empty()) {
		std::cerr << "No cameras available.\n";
		return;
	}

	using namespace std::chrono_literals;
	//remember that cameras is a vector of std::unique_ptr<camera< ...>> this could be more than one camera
	auto resolution_opt = cameras.at(0)->getEventResolution();
	auto event_store = cameras.at(0)->getEventStore();
	if (!resolution_opt.has_value()) {
		std::cerr << "Camera resolution not set.\n";
		return;
	}

	dv::visualization::EventVisualizer visualizer(*resolution_opt);
	visualizer.setBackgroundColor(dv::visualization::colors::white);
	visualizer.setPositiveColor(dv::visualization::colors::iniBlue);
	visualizer.setNegativeColor(dv::visualization::colors::darkGrey);

	cv::namedWindow("Events", cv::WINDOW_AUTOSIZE);

	const int64_t start_ts = event_store -> getLowestTime();
	const int64_t end_ts = event_store -> getHighestTime();
	const int64_t frame_interval = 100;  // µs (10ms)

	for (int64_t t = start_ts; t < end_ts; t += frame_interval) {
		// Slice events in current interval
		auto sliced_events = event_store -> sliceTime(t, t + frame_interval);

		if (!sliced_events.isEmpty()) {
			cv::Mat frame = visualizer.generateImage(sliced_events);
			cv::imshow("Events", frame);
			cv::waitKey(0);  // Adjust for playback speed 
		}
	}
	cv::waitKey(0);  // Wait at end to keep window open
}

template<std::int8_t Dimensions>
void Scene<Dimensions>::visualizeFrames() const {
	//debugging
	std::vector<cv::Mat>* frames = cameras.at(0)->getFrameStore();
	for (std::size_t i = 0; i < frames->size(); ++i) {
		std::string window_name = std::to_string(i);
		cv::imshow(window_name, frames->at(i));
		cv::waitKey(0);  // Or use 0 to pause until keypress
		cv::destroyWindow(window_name);  // Close after showing
		++i;
	}
}

template<std::int8_t Dimensions>
void Scene<Dimensions>::start_simulation(
	Scene_constants::translation t_mode, //need another typename as placeholder because dimension at compile time not known. 
	Scene_constants::velocity v_mode,
	Scene_constants::camera_para store_mode)
{
	std::int64_t current_time = 0;
	const std::int64_t time_step = 10000;   // µs
	const std::int64_t end_time = 1000000;  // µs, i.e., 1ms


	
	std::int16_t i{ 1 };
	while (current_time < end_time) {
		// 1. Move all objects
		
		for (auto& obj : objects) {
			//debug
			std::cout << "Object Center Position " << obj->getObjPosition() << '\n';
			obj->transform(time_step, t_mode, v_mode);
		}
		std::cout << "Frame number " << i << " is captured " << '\n';
		++i;

		//Debug
		//std::cout << "iteration" << i << '\n';
		//std::cout << "number of cameras" << cameras.size() << '\n';
		//std::cout << "number of objects" << objects.size() << '\n';
		
		
		// For each camera, cast rays

		// for Frame visualtisatoin
		//std::int16_t frame_counter{1};

		

		for (const auto& camera : cameras) {
			const auto& rays = camera->get_rays_dynamic();  // use polymorphic methods. Template Methods can not be made virtual. Therefore necessary to change 
			// to Eigen::Dyanmic here. 
			const Eigen::Vector3d cam_pos = camera->get_camera_pos();
			dv::EventStore* event_store{ camera->getEventStore() };

			const int height = rays.rows();
			const int width = rays.cols();

			//debugging
			cv::Mat visibility_frame = cv::Mat::zeros(height, width, CV_8UC1);  // single-channel grayscale image

			

			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					const Eigen::Vector3d ray_dir = rays(y, x);
					Eigen::Vector3d hit, normal;
					double distance;
					std::int16_t brightness{ 0 };
					Eigen::Matrix<Eigen::Vector3d, Eigen::Dynamic, Eigen::Dynamic> camera_picture{};
					for (const auto& obj : objects) {
						if (obj->intersect(cam_pos, ray_dir, hit, normal, distance)) {
							//debugg
							//std::cout << "Hit detected !";
							visibility_frame.at<uchar>(y, x) = 255;  // Mark hit as white


							brightness = obj->calculate_brightness(hit, normal, cam_pos, Scene_constants::lightning::lambertian_basic);
							//visibility_frame.at<uchar>(y, x) = int(brightness * 255);
							//pos_events
						}
						std::optional<bool> polarity = camera->check_brightness_change(x, y, brightness);
							
						if (polarity.has_value()) {
							//(polarity.value()) ? std::cout << "pos Event entdeckt \n" : std::cout << "neg event entedeckt. \n";
							//std::cout << "Event detected ! " << '\n'; 
							event_store->emplace_back(current_time, x, y, polarity.value());
								
							
							
						}
					}
				}
			}
			if (static_cast<int>(store_mode)) {
				std::vector<cv::Mat>* frames{ camera->getFrameStore() };
				frames->emplace_back(visibility_frame);
			}
			
 		}

		current_time += time_step;
	}

	// TODO: Save or visualize `event_store`
}
