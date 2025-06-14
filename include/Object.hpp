
//Base class for simulation of Events 
class Object3D {
public:
    virtual ~Object3D() = default;

    // Ray intersection: returns true if hit, and fills hit point & normal
    virtual bool intersect(
        const Eigen::Vector3d& ray_origin,
        const Eigen::Vector3d& ray_direction,
        Eigen::Vector3d& hit_point,
        Eigen::Vector3d& surface_normal,
        double& distance
    ) = 0;

    virtual void transform(std::int64_t timestamp, Scene::translation movement_mode, Scene) = 0;
    virtual float calculate_brightness(const Eigen::Vector3d& hit_point, const Eigen::Vector3d& normal) const = 0;
    virtual bool check_brightness_change(int x, int y, float new_brightness) = 0;
};
