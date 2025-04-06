#pragma once
#include "amr_localization/map.hpp"
#include <tuple>
#include <vector>
#include <memory>
#include <limits>
#include <cmath>
#include <random>

struct Particle {
    double x;
    double y;
    double theta;
    double weight;

    Particle(double x_, double y_, double theta_, double weight_ = 1.0)
        : x(x_), y(y_), theta(theta_), weight(weight_) {}
};

class ParticleFilter {
public:
    ParticleFilter(
        double dt,
        const std::string& map_path,
        int particle_count,
        double sigma_v = 0.05,
        double sigma_w = 0.1,
        double sigma_z = 0.2,
        double sensor_range_max = 8.0,
        double sensor_range_min = 0.16,
        bool global_localization = true,
        std::tuple<double, double, double> initial_pose = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()},
        std::tuple<double, double, double> initial_pose_sigma = {
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::quiet_NaN()}
    )
        : _dt(dt),
          _initial_particle_count(particle_count),
          _particle_count(particle_count),
          _sensor_range_max(sensor_range_max),
          _sensor_range_min(sensor_range_min),
          _sigma_v(sigma_v),
          _sigma_w(sigma_w),
          _sigma_z(sigma_z),
          _iteration(0)
    {
        _map = std::make_shared<Map>(map_path, sensor_range_max, 0.08, true, false);
        _particles = _init_particles(particle_count, global_localization, initial_pose, initial_pose_sigma);
    }

    std::tuple<bool, std::tuple<double, double, double>> compute_pose();

    void move(double v, double w);

    void resample(const std::vector<double>& measurements);

private:
    std::vector<Particle> _init_particles(
        int particle_count,
        bool global_localization,
        const std::tuple<double, double, double>& initial_pose,
        const std::tuple<double, double, double>& initial_pose_sigma
    );

    std::vector<double> _sense(const Particle& particle);

    static double _gaussian(double mu, double sigma, double x);

    std::vector<std::vector<std::pair<double, double>>> _lidar_rays(
        const std::tuple<double, double, double>& pose,
        const std::vector<int>& indices,
        double degree_increment = 1.5
    );

    double _measurement_probability(
        const std::vector<double>& measurements,
        const Particle& particle
    );

    // Attributes
    double _dt;
    int _initial_particle_count;
    int _particle_count;
    double _sensor_range_max;
    double _sensor_range_min;
    double _sigma_v;
    double _sigma_w;
    double _sigma_z;
    int _iteration;

    std::shared_ptr<Map> _map;
    std::vector<Particle> _particles;
    std::string _timestamp;
};
