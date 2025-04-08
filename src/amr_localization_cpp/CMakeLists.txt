cmake_minimum_required(VERSION 3.8)
project(amr_localization_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(amr_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(nlohmann_json REQUIRED)

find_package(mlpack REQUIRED)
find_package(Armadillo REQUIRED)
find_package(GEOS REQUIRED)

# Add C++ "include" directory
include_directories(include
  ${Armadillo_INCLUDE_DIRS}
  ${mlpack_INCLUDE_DIRS}
)

# DE AQUI ES CHAT
# Añadir biblioteca compartida desde intersect.c
# add_library(intersect SHARED src/intersect.c)
# target_include_directories(intersect PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)


# Crear ejecutable del nodo particle_filter_node
add_executable(particle_filter_node
  src/particle_filter_node.cpp
  src/particle_filter.cpp
  src/intersect.c
  src/maps.cpp
)

# Linkea con la lib compartida y dependencias ROS
# target_link_libraries(particle_filter_node intersect)
# HASTA AQUI ES CHAT

ament_target_dependencies(particle_filter_node 
  rclcpp 
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  amr_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  message_filters
  nlohmann_json
  Eigen3
)

target_link_libraries(particle_filter_node
  ${Armadillo_LIBRARIES}
  ${mlpack_LIBRARIES}
)

target_include_directories(particle_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Armadillo_INCLUDE_DIRS}
  ${mlpack_INCLUDE_DIRS}
)

# Install C++ executables
install(TARGETS
  particle_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
