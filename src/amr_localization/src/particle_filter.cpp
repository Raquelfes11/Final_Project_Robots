#include "amr_localization/map.hpp"

// C++ built-in libraries
#include <cmath>
#include <list>
#include <random>   // Built-in module for random number generation and distribution sampling
#include <vector>   // Allows using std::vector
#include <tuple>
#include <algorithm>

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
        localized: bool = False
        pose: tuple[float, float, float] = (float("inf"), float("inf"), float("inf"))

        complex_particles = np.zeros((self._particles.shape[0], 4))
        complex_particles[:, :2] = self._particles[:, :2]
        complex_particles[:, 2] = np.cos(self._particles[:, 2].astype(float))
        complex_particles[:, 3] = np.sin(self._particles[:, 2].astype(float))

        clustering = DBSCAN(eps=0.1, min_samples=25).fit(complex_particles)

        labels = clustering.labels_
        num_clusters = len(set(labels) - {-1})

        if num_clusters == 1:
            localized = True

            particles = complex_particles[labels != -1]

            # Proyectamos los angulos sobre el plano complejo
            pose = (
                np.mean(particles[:, 0]),
                np.mean(particles[:, 1]),
                np.arctan2(np.mean(particles[:, 3]), np.mean(particles[:, 2])),
            )

            idxs = np.random.choice(len(self._particles), 50, replace=False)
            self._particles = self._particles[idxs]
        else:
            new_particles = []
            for label in labels:
                particles = complex_particles[labels == label]
                num_particles = len(particles)
                idxs = np.random.choice(
                    num_particles, int(num_particles * 0.85), replace=False
                )  # We want to keep only the 85% of the particles

                new_particles.append(particles[idxs])

        return localized, pose
    }

    void move(double v, double w)
    {
        _iteration++;

        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<> noise_v{0, _sigma_v};
        std::normal_distribution<> noise_w{0, _sigma_w};

        double noise, x, y, theta;
        for (auto &particle : _particles)
        {
            theta = std::get<2>(particle);

            noise = noise_v(generator);
            x = std::get<0>(particle) + (v + noise) * std::cos(theta) * _dt;
            y = std::get<1>(particle) + (v + noise) * std::sin(theta) * _dt;
            theta = theta + (w + noise_w(generator)) * _dt;
            theta = std::fmod(theta_new, 2 * M_PI);

            if (_map->contains(x, y))
            {
                particle = std::make_tuple(x, y, theta);
            }
            else
            {
                auto collision_point;
                std::tie(collision_point, std::ignore) = _map->check_collision({x, y}, {x_new, y_new});
                particle = std::make_tuple(collision_point.first, collision_point.second, theta_new);
            }
        }
    }

    void resample(const std::vector<double>& measurements)
    {
        std::vector<double> weights;
        for (const auto& particle : _particles)
        {
            std::list<double> downsampled_measurements;
            for (size_t i = 0; i < measurements.size(); i += 30)
            {
                downsampled_measurements.push_back(measurements[i]);
            }
            
            double w = this->_measurement_probability(
                std::vector<double>(downsampled_measurements.begin(), downsampled_measurements.end()),
                particle
            );

            weights.push_back(w);
        }

        // Normalize weights
        double sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
        for (double& w : weights)
        {
            w /= sum_weights;
        }

        // Resample using stochastic resampling
        

        num_weights = len(weights)

        uniforms = np.random.rand() / num_weights + np.arange(num_weights) / num_weights
        indices = np.digitize(uniforms, bins=np.cumsum(weights))

        self._particles = self._particles[indices]
        
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
        for (int i = 0; i < particle_count; ++i)
        {
            if (global_localization)
            {
                do {
                    x = uniform_x(gen);
                    y = uniform_y(gen);
                } while (!_map->contains(x, y));
                theta = uniform_theta(generator) * M_PI / 2;
            }
            else
            {
                do {
                    x = normal_x(gen);
                    y = normal_y(gen);
                } while (!_map->contains(x, y));
                theta = normal_theta(generator) * M_PI / 2;
            }
            std::tuple<double, double, double> particle{x, y, theta};

            _particles.push_back(particle);
        }

        return _particles
    }

    std::vector<double> _sense(const std::tuple<double, double, double>& particle)
    {
        std::vector<double> z_hat;

        std::vector<int> idxs;
        for (int i = 0; i <= 240; i += 30)
        {
            indices.push_back(i);
        }

        auto sim_ranges = this->_lidar_rays(particle, idxs);

        double collision_dist;
        for (const auto& ray : sim_ranges) {
            std::tie(std::ignore, collision_dist) = _map->check_collision(ray, compute_distance=true);
            z_hat.push_back(collision_dist);
        }

        return z_hat
    }

    static double _gaussian(double mu, double sigma, double x)
    {
        double exponent = -0.5 * std::pow((x - mu) / sigma, 2);
        double denominator = std::sqrt(2 * M_PI * sigma * sigma);
        return std::exp(exponent) / denominator;
    }

    std::vector<std::vector<std::pair<double, double>>> _lidar_rays(
        const std::tuple<double, double, double>& pose,
        const std::vector<int>& indices,
        double degree_increment = 1.5
    )
    {
        double x, y, theta;
        std::tie(x, y, theta) = pose;

        // Convert the sensor origin to world coordinates
        double x_start = x - 0.035 * std::cos(theta);
        double y_start = y - 0.035 * std::sin(theta);

        std::vector<std::vector<std::pair<double, double>>> rays;

        for (int index : indices)
        {
            double ray_angle = degree_increment * index * M_PI / 180.0; // radians
            double x_end = x_start + self._sensor_range_max * std::cos(theta + ray_angle);
            double y_end = y_start + self._sensor_range_max * std::sin(theta + ray_angle);
            rays.push_back({{x_start, y_start}, {x_end, y_end}});
        }

        return rays
    }

    double _measurement_probability(
        const std::vector<double>& measurements,
        const std::tuple<double, double, double>& particle
    )
    {
        double probability = 1.0;

        auto predicted_measurements = this->_sense(particle);
        for (size_t i = 0; i < measurements.size(); ++i)
        {
            double z = std::isnan(measurements[i]) ? _sensor_range_min : measurements[i];
            double z_hat = std::isnan(predicted[i]) ? _sensor_range_min : predicted[i];
            probability *= this->_gaussian(z_hat, _sigma_z, z);
        }

        return probability;
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