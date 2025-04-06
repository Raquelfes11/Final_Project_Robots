#include "amr_localization/map.hpp"

// C++ built-in libraries
#include <cmath>
#include <random>   // Built-in module for random number generation and distribution sampling
#include <vector>   // Allows using std::vector

// Third-party libraries
#include <Eigen/Dense> // Allows using Eigen matrices

class ParticleFilter
{
public:
    ParticleFilter(
        double dt,
        const std::string &map_path,
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
    {
        _dt = dt;
        _initial_particle_count = particle_count;
        _particle_count = particle_count;
        _sensor_range_max = sensor_range_max;
        _sensor_range_min = sensor_range_min;
        _sigma_v = sigma_v;
        _sigma_w = sigma_w;
        _sigma_z = sigma_z;
        _iteration = 0;

        _map = std::make_shared<Map>(map_path, sensor_range_max, true, false, 0.08);
        _particles = _init_particles(particle_count, global_localization, initial_pose, initial_pose_sigma);
    }

    // Public Methods
    std::tuple<bool, std::tuple<double, double, double>> compute_pose()
    {
        // TODO
    }

    void move(double v, double w)
    {
        _iteration += 1;

        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution noise_v{0, _sigma_v};
        std::normal_distribution noise_w{0, _sigma_w};

        double noise, x, y;
        for (auto &particle : _particles)
        {
            noise = noise_v(generator);
            x = particle[0] + (v + noise)*std::cos(particle[2])*_dt;
            y = particle[1] + (v + noise)*std::sin(particle[2])*_dt;
            particle[2] += (w + noise_w(generator))*_dt;
            particle[2] %= 2*M_PI;

            if (_map->contains(x, y))
            {
                particle[0] = x;
                particle[1] = y;

                // TODO
            }
        }
    }

    void resample(const std::vector<double>& measurements)
    {
        // TODO
    }

private:
    // Private Methods
    std::tuple<double, double, double> _init_particles(
        int particle_count,
        bool global_localization,
        const std::tuple<double, double, double>& initial_pose,
        const std::tuple<double, double, double>& initial_pose_sigma
    )
    {
        double x_min, y_min, x_max, y_max;
        std::tie(x_min, y_min, x_max, y_max) = _map._map_polygon.bounds;

        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_real_distribution<> uniform_x{x_min, x_max};
        std::uniform_real_distribution<> uniform_y{y_min, y_max};
        std::uniform_int_distribution<int> uniform_theta(0, 3);

        std::normal_distribution normal_x{initial_pose[0], initial_pose_sigma[0]};
        std::normal_distribution normal_y{initial_pose[1], initial_pose_sigma[1]};
        std::normal_distribution normal_theta{initial_pose[2], initial_pose_sigma[2]};

        double x, y, theta;
        for (int i=0; i < particle_count; ++i)
        {
            if (global_localization)
            {
                x = uniform_x(generator);
                y = uniform_y(generator);
                while (!_map->contains(x, y))
                {
                    x = uniform_x(generator);
                    y = uniform_y(generator);
                }
                theta = uniform_theta(generator) * M_PI / 2;
            }
            else
            {
                x = normal_x(generator);
                y = normal_y(generator);
                while (!_map->contains(x, y))
                {
                    x = normal_x(generator);
                    y = normal_y(generator);
                }
                theta = normal_theta(generator) * M_PI / 2;
            }
            std::tuple<double, double, double> particle{x, y, theta};

            _particles.push_back(particle);
        }
    }

    std::vector<double> _sense(const std::tuple<double, double, double>& particle)
    {
        // TODO
    }

    static double _gaussian(double mu, double sigma, double x)
    {
        // TODO
    }

    std::vector<std::vector<std::pair<double, double>>> _lidar_rays(
        const std::tuple<double, double, double>& pose,
        const std::vector<int>& indices,
        double degree_increment = 1.5
    )
    {
        // TODO
    }

    double _measurement_probability(
        const std::vector<double>& measurements,
        const std::tuple<double, double, double>& particle
    )
    {
        // TODO
    }

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
    std::vector<std::tuple<double, double, double> _particles;

}