std::tuple<bool, std::tuple<double, double, double>> compute_pose(const std::vector<double>& measurements)
{
    std::vector<double> weights;
    weights.reserve(_particles.size());

    // 1. Calcular pesos usando medidas reales contra medidas simuladas
    for (const auto& particle : _particles)
    {
        std::list<double> downsampled_measurements;
        for (size_t i = 0; i < measurements.size(); i += 30)
        {
            downsampled_measurements.push_back(measurements[i]);
        }

        double w = _measurement_probability(
            std::vector<double>(downsampled_measurements.begin(), downsampled_measurements.end()),
            particle
        );

        weights.push_back(w);
    }

    // 2. Normalizar los pesos
    double sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
    for (double& w : weights)
    {
        w /= sum_weights;
    }

    // 3. Resample usando muestreo estocástico universal
    std::vector<std::tuple<double, double, double>> new_particles;
    new_particles.reserve(_particles.size());

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0.0, 1.0 / _particles.size());

    double r = dist(gen);
    double c = weights[0];
    int i = 0;

    for (size_t m = 0; m < _particles.size(); ++m)
    {
        double U = r + m * (1.0 / _particles.size());
        while (U > c)
        {
            i++;
            if (i >= static_cast<int>(weights.size())) {
                i = weights.size() - 1;
                break;
            }
            c += weights[i];
        }
        new_particles.push_back(_particles[i]);
    }

    _particles = new_particles;

    // 4. NO calculamos el promedio del pose aquí (esa lógica está en `resample`)
    // Devuelve dummy pose (no-localizado) por ahora
    return std::make_tuple(false, std::make_tuple(std::numeric_limits<double>::quiet_NaN(),
                                                  std::numeric_limits<double>::quiet_NaN(),
                                                  std::numeric_limits<double>::quiet_NaN()));
}
