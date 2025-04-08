#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <mlpack/methods/dbscan/dbscan.hpp>
#include <armadillo>
#include <cmath>
#include <list>
#include <random>
#include <vector>
#include <tuple>

#include <algorithm>

#include <Eigen/Dense>  // Si es necesario en el proyecto

#include "amr_localization_cpp/maps.hpp"

class ParticleFilter
{
public:
    // Constructor
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
        std::tuple<double, double, double> initial_pose = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()},
        std::tuple<double, double, double> initial_pose_sigma = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}
    );

    // Métodos públicos
    std::tuple<bool, std::tuple<double, double, double>> compute_pose();
    void move(double v, double w);
    void resample(const std::vector<double>& measurements);

private:
    // Métodos privados
    std::tuple<double, double, double> _init_particles(
        int particle_count,
        bool global_localization,
        const std::tuple<double, double, double>& initial_pose,
        const std::tuple<double, double, double>& initial_pose_sigma
    );

    std::vector<double> _sense(const std::tuple<double, double, double>& particle);
    
    static double _gaussian(double mu, double sigma, double x);
    
    std::vector<std::vector<std::pair<double, double>>> _lidar_rays(
        const std::tuple<double, double, double>& pose,
        const std::vector<int>& indices,
        double degree_increment = 1.5
    );
    
    double _measurement_probability(
        const std::vector<double>& measurements,
        const std::tuple<double, double, double>& particle
    );

    // Atributos privados
    double _dt;
    int _initial_particle_count;
    int _particle_count;
    double _sensor_range_max;
    double _sensor_range_min;
    double _sigma_v;
    double _sigma_w;
    double _sigma_z;
    int _iteration;

    Map _map;
    std::vector<std::tuple<double, double, double>> _particles;
};

#endif // PARTICLE_FILTER_HPP
