#ifndef MAP_HPP
#define MAP_HPP

#include <nlohmann/json.hpp> // Para el manejo de JSON
#include <Eigen/Dense>       // Para usar Eigen, que es necesario para matrices
#include <vector>
#include <string>
#include <memory>
#include <geos_c.h> // Librería GEOS
#include <optional>
#include <tuple>
#include <stdexcept>
#include <cmath>
#include <iostream>

namespace fs = std::filesystem;

using json = nlohmann::json;

class Map {
private:
    GEOSGeometry* _map_polygon; // Polígono que representa el mapa
    GEOSGeometry* _safe_map_polygon; // Polígono que representa el área segura
    std::vector<std::vector<std::pair<double, double>>> _map_segments; // Segmentos del mapa
    Eigen::MatrixXi _grid_map; // Matriz del mapa de cuadrícula
    GEOSContextHandle_t _geos_ctx; // Contexto GEOS

    float _sensor_range; // Rango del sensor
    void* _intersect; // Para las intersecciones con GEOS
    std::vector<std::vector<std::vector<std::pair<double, double>>>> _region_segments;

    int _XC; // Fila para las regiones
    int _YR; // Columna para las regiones

    std::vector<std::pair<double, double>> _boundary; // Frontera del mapa
    std::vector<std::vector<std::pair<double, double>>> _holes; // Agujeros dentro del mapa

    void* _init_intersect(); // Inicializa la intersección utilizando una librería dinámica
    GEOSGeometry* GeoGeoFromWKT(const std::string& wkt, GEOSContextHandle_t ctx); // Convierte WKT a GEOSGeometry
    
    std::string generate_wkt_from_boundary_and_holes(); // Genera WKT de la frontera y los agujeros
    
    std::optional<std::pair<double, double>> segment_intersect(
        const std::pair<double, double>& p1,
        const std::pair<double, double>& p2,
        const std::pair<double, double>& q1,
        const std::pair<double, double>& q2
    ); // Verifica si dos segmentos se intersectan
    
    std::pair<std::pair<double, double>, double> check_collision(
        const std::pair<double, double>& start,
        const std::pair<double, double>& end,
        bool compute_distance = false
    ); // Verifica si hay colisiones entre segmentos
    
    std::vector<std::vector<std::vector<std::pair<double, double>>>> _init_regions(); // Inicializa las regiones
    
    std::pair<int, int> _xy_to_rc(double x, double y) const; // Convierte coordenadas a índices de la región

public:
    Map(
        const std::string& json_file,
        float sensor_range,
        float safety_distance = 0.0,
        bool use_regions = true,
        bool compiled_intersect = true
    ); // Constructor de la clase Map

    ~Map(); // Destructor

    bool contains(double x, double y); // Verifica si un punto está dentro del mapa
    bool crosses(double x1, double y1, double x2, double y2); // Verifica si una línea cruza el mapa
};

#endif // MAP_HPP
