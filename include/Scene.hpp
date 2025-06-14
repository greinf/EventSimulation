#include <Eigen/Dense>
#include <iostream>
#include <_Camera.hpp>
#include <Surface.hpp>
#include <Object.hpp>
#include <vector>
#include <memory>

template<std::int8_t Dimensions>
class Scene {  //WIDTH and HEIGHT no compile time constants. 
public:
	Scene(std::size_t n_cameras, std::size_t n_objects) :
		timing{ 0 }
	{
		cameras.reserve(n_cameras);
		objects.reserve(n_objects);
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
		objects.emplace_back(std::move(obj));
	}

	template<std::int16_t WIDTH, std::int16_t HEIGHT>
	void addcamera(std::unique_ptr<Camera<WIDTH, HEIGHT>> camera) {
		cameras.emplace_back(std::move(camera));
	}

	std::int64_t timing{};
	void start_simulation(typename Scene<Dimension>::translation t_mode, typename Scene<Dimension>::velocity v_mode); //for starting of timer and propably moving the object 

	enum class translation {
		frame_static,
		translation_right,
		tranlsation_left,
		rotation_clockwise,
		rotation_c_clockwise,
		max_elemtens,
	};

	enum class velocity {
		slow,
		medium,
		fast,
		max_elements,
	};
	
private:
	static constexpr std::int8_t width = Dimensions;
	std::vector < std::unique_ptr<_Camera>> cameras{};
	std::vector < std::unique_ptr<Object3D>> obj{};

};




template<std::int8_t Dimensions>
void Scene<Dimensions>::start_simulation(
	typename Scene<Dimensions>::translation t_mode, //need another typename as placeholder because dimension at compile time not known. 
	typename Scene<Dimensions>::velocity v_mode)
{
	std::int64_t current_time = 0;
	const std::int64_t time_step = 10;   // µs
	const std::int64_t end_time = 1000;  // µs, i.e., 1ms

	dv::EventStore event_store;

	while (current_time < end_time) {
		// 1. Move all objects
		for (auto& obj : objects) {
			obj->transform(current_time, t_mode, v_mode);
		}

		// 2. For each camera, cast rays
		for (const auto& camera : cameras) {
			const auto& rays = camera->get_rays_dynamic();  // use polymorphic method
			const Eigen::Vector3d cam_pos = camera->get_camera_pos();

			const int height = rays.rows();
			const int width = rays.cols();

			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					const Eigen::Vector3d ray_dir = rays(y, x);
					Eigen::Vector3d hit, normal;
					double distance;

					for (const auto& obj : objects) {
						if (obj->intersect(cam_pos, ray_dir, hit, normal, distance)) {
							float brightness = obj->calculate_brightness(hit, normal);
							bool polarity = obj->check_brightness_change(x, y, brightness);

							if (polarity != 0) {
								dv::Event e(x, y, current_time, polarity);
								event_store.push_back(e);
							}
						}
					}
				}
			}
		}

		current_time += time_step;
	}

	// TODO: Save or visualize `event_store`
}
