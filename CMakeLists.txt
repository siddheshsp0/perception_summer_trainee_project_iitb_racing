cmake_minimum_required(VERSION 3.8)
project(perception_winter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




# ---------------------------- Added by the author ---------------------------

# Adding global dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Adding executables
add_executable(process_lidar
  src/process_lidar.cpp
  src/main.cpp)
ament_target_dependencies(process_lidar rclcpp sensor_msgs)


# Install targets section (so that ros2 run can find your nodes)
install(TARGETS
  process_lidar
  DESTINATION lib/${PROJECT_NAME}
)

# Adding other directories
include_directories(include) # Tells compiler where to look for header files

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# ----------------------------- END -----------------------










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

ament_package()
