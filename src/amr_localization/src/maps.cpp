// Libraries
// Ctypes // TODAVIA NO ESTOY SEGURA DE CUAL SERIA BUENA PARA CTYPES
#include <nlohmann/json.hpp> // Json
#include <cmath> // Math
#include <Eigen/Dense> // Numpy
#include <iostream> //Os
#include <cstdlib> // Platform
#include <matplotlibcpp.h> // Matplotlib
#include <geos_c.h> //Shapely

#include <fstream>
#include <vector>
#include <array>
#include <filesystem>
#include <stdexcept>

// CREO QUE HACE FALTA IMPORTAR ESO ASÍ PARA PODER USARLO
using json = nlohmann::json;
namespace plt = matplotlibcpp;
namespace fs = std::filesystem;

class Map
{

private:
    // Rellenar
    GEOSGeometry* _map_polygon; // ESTO VA A SER EL POLÍGONO DE LA LIBRERÍA GEOS QUE REPRESENTE EL MAPA
    GEOSGeometry* _safe_map_polygon; // ESTO VA A SER EL POLÍGONO QUE REPRESENTE EL ÁREA SEGURA
    std::vector<std::vector<std::pair<double, double>>> _map_segments; // ESTO VAN A SER LOS EDGES
    Eigen::MatrixXi _grid_map; // GRID MAP
    GEOSContextHandle_t _geos_ctx = GEOS_init_r();

    float _sensor_range;       
    void* _intersect; // PARA LAS INTERSECCIONES TMB VAMOS A USAR GEOS       
    std::vector<std::vector<std::vector<std::pair<double, double>>>> _region_segments;

    // PARA LOS ÚLTIMOS MÉTODOS
    int _XC;
    int _YR;

    void* _init_intersect() {
        std::string library_name;

    // HE PUESTO ESTO PARA QUE FUNCIONE BIEN CUANDO LO PROBEIS
    #ifdef _WIN32
        library_name = "libintersect.dll";
    #elif __APPLE__
            library_name = "libintersect.dylib";
    #else
            library_name = "libintersect.so";
    #endif
        std::string library_path = fs::current_path().string() + "/" + library_name;
            void* handle = dlopen(library_path.c_str(), RTLD_LAZY);
            if (!handle) {
                throw std::runtime_error("Could not load library: " + library_path);
            }

            return handle;
    }

    // CHICAS ESTA QUE VIENE A CONTINUACIÓN NO HE SABIDO HACERLA
    std::vector<std::vector<std::vector<std::pair<double, double>>>> _init_regions() {
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::lowest();

        for (const auto& pt : _boundary) {
            x_min = std::min(x_min, pt.first);
            x_max = std::max(x_max, pt.first);
            y_min = std::min(y_min, pt.second);
            y_max = std::max(y_max, pt.second);
        }

        int map_rows = std::ceil(y_max - y_min);
        int map_cols = std::ceil(x_max - x_min);

        _XC = std::floor(map_cols / 2.0);
        _YR = map_rows - std::ceil(map_rows / 2.0);

        std::vector<std::vector<std::vector<std::pair<double, double>>>> region_segments(
            map_rows, std::vector<std::vector<std::pair<double, double>>>(map_cols));

        for (double y = y_max - 0.5; y > y_min; y -= 1.0) {
            for (double x = x_min + 0.5; x < x_max; x += 1.0) {
                GEOSGeometry* pt = GEOSGeom_createPointFromXY_r(_geos_ctx, x, y);
                GEOSGeometry* circle = GEOSBuffer_r(_geos_ctx, pt, _sensor_range + 1.0 / std::sqrt(2), 8);
                GEOSGeom_destroy_r(_geos_ctx, pt);

                std::vector<std::pair<double, double>> visible_segments;

                for (const auto& segment : _map_segments) {
                    GEOSCoordSequence* seq = GEOSCoordSeq_create_r(_geos_ctx, 2, 2);
                    GEOSCoordSeq_setX(seq, 0, segment[0].first);
                    GEOSCoordSeq_setY(seq, 0, segment[0].second);
                    GEOSCoordSeq_setX(seq, 1, segment[1].first);
                    GEOSCoordSeq_setY(seq, 1, segment[1].second);
                    GEOSGeometry* line = GEOSGeom_createLineString_r(_geos_ctx, seq);

                    if (GEOSIntersects_r(_geos_ctx, line, circle) && !GEOSTouches_r(_geos_ctx, line, circle)) {
                        visible_segments.push_back(segment[0]);
                        visible_segments.push_back(segment[1]);
                    }

                    GEOSGeom_destroy_r(_geos_ctx, line);
                }

                int r, c;
                std::tie(r, c) = _xy_to_rc(x, y);
                region_segments[r][c] = visible_segments;

                GEOSGeom_destroy_r(_geos_ctx, circle);
            }
        }

        return region_segments;
    }

    std::pair<int, int> _xy_to_rc(double x, double y) const {
        int row = std::max(0, static_cast<int>(_YR - std::ceil(y)));
        int col = std::max(0, static_cast<int>(std::floor(x) + _XC));
        return {row, col};
    }


    // ESTO LO VAMOS A UTILIZAR PARA MÁS ADELANTE
    std::vector<std::pair<double, double>> _boundary;
    std::vector<std::vector<std::pair<double, double>>> _holes;


public:
    // Rellenar
    Map(
        const std::string& json_file,
        float sensor_range,
        float safety_distance = 0.0,
        bool use_regions = true,
        bool compiled_intersect = true
    ) : _sensor_range(sensor_range){

        // CARGAMOS EL MAPA DEL ARCHIVO JSON
        // Load map from JSON file
        std::ifstream file(json_file);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open JSON file");
        }
        json data;
        file >> data;

        // CARGAMOS LOS BOUNDARY Y HOLES
        auto boundary = data["metric"]["boundary"];
        auto holes = data["metric"]["holes"];

        for (const auto& pt : boundary) {
            _boundary.emplace_back(pt[0], pt[1]);
        }

        for (const auto& hole : holes) {
            std::vector<std::pair<double, double>> h;
            for (const auto& pt : hole) {
                h.emplace_back(pt[0], pt[1]);
            }
            _holes.push_back(h);
        }

        // CREAMOS EL MAPA POLYGON, USANDO GEOS
        // Create a polygon map
        _map_polygon = GeoGeoFromWKT("POLYGON((...))");
        // CHAT ME HA DOCHO QUE ESTA LINEA DE ARRIBA ESTARIA MAL, QUE PONGA ESTO
        //GEOSGeometry* GeoGeoFromWKT(const std::string& wkt, GEOSContextHandle_t ctx) {
        //GEOSWKTReader* reader = GEOSWKTReader_create_r(ctx);
        //GEOSGeometry* geom = GEOSWKTReader_read_r(ctx, reader, wkt.c_str());
        //GEOSWKTReader_destroy_r(ctx, reader);
        //return geom;
        //}
        //std::string polygon_wkt = "...";  // Genera el WKT dinámicamente desde _boundary y _holes
        //_map_polygon = GeoGeoFromWKT(polygon_wkt, _geos_ctx);

        // AHORA CREAMOS UN MAPA MÁS SEGURO ENGORDANDO LOS OBSTÁCULOS CON GEOS
        // Create a safe map by enlarging obstacles
        //_safe_map_polygon = GeoGeoBuffer(_map_polygon, -safety_distance, true);
        // ESTO QUIZAS TMB
        _safe_map_polygon = GEOSBuffer_r(_geos_ctx, _map_polygon, -safety_distance, 8);
        //}


        // CREAMOS LOS SEGMENTOS
        // Create a segment map
        for (size_t i = 0; i < boundary.size() - 1; ++i) {
            _map_segments.push_back({boundary[i], boundary[i + 1]});
        }
        _map_segments.push_back({boundary.back(), boundary.front()});

        // AHORA LA PARTE DE LOS HOLES
        for (const auto& hole : holes) {
            for (size_t i = 0; i < hole.size() - 1; ++i) {
                _map_segments.push_back({hole[i], hole[i + 1]});
            }
            _map_segments.push_back({hole.back(), hole.front()});
        }

        // CREAMOS EL MAPA GRID SI LO TENEMOS GUARDADO EN ELA RCHIVO JSON
        // Create a grid map
        try {
            auto map_size = data["grid"]["size"];
            auto obstacles = data["grid"]["obstacles"];
            
            // MATRIZ NxM
            _grid_map = Eigen::MatrixXi::Zero(map_size[0], map_size[1]);

            for (const auto& obstacle : obstacles) {
                _grid_map(obstacle[0], obstacle[1]) = 1;
            }
        } catch (const std::exception& e) {
            // HANDLE MISSING
            _grid_map = Eigen::MatrixXi();
        }

        // Performance optimizations
        _intersect = compiled_intersect ? _init_intersect() : nullptr;
        _region_segments = use_regions ? _init_regions() : nullptr;
    }
    void show(const std::string& title) {
        // TODO: Convertir los segmentos a coordenadas X e Y para plotear
        plt::title("Map (" + title + ")");
        plt::xlabel("x [m]");
        plt::ylabel("y [m]");
        plt::show();
    }

    bool contains(double x, double y) {
        GEOSGeometry* pt = GEOSGeom_createPointFromXY_r(_geos_ctx, x, y);
        char res = GEOSContains_r(_geos_ctx, _safe_map_polygon, pt);
        GEOSGeom_destroy_r(_geos_ctx, pt);
        return res == 1;
    }

    bool crosses(double x1, double y1, double x2, double y2) {
        GEOSCoordSequence* seq = GEOSCoordSeq_create_r(_geos_ctx, 2, 2);
        GEOSCoordSeq_setX(seq, 0, x1);
        GEOSCoordSeq_setY(seq, 0, y1);
        GEOSCoordSeq_setX(seq, 1, x2);
        GEOSCoordSeq_setY(seq, 1, y2);
        GEOSGeometry* line = GEOSGeom_createLineString_r(_geos_ctx, seq);
        char res = GEOSIntersects_r(_geos_ctx, _safe_map_polygon, line);
        GEOSGeom_destroy_r(_geos_ctx, line);
        return res == 1;
    }
};

int main() {
    std::string map_name = "lab03";
    std::string map_path = fs::current_path().string() + "/../maps/" + map_name + ".json";

    Map m(map_path, 1.0);
    m.show(map_name);

    return 0;
}