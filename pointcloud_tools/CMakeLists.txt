# cmake_minimum_required(VERSION 3.5)
# project(pointcloud)
# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(pcl_conversions REQUIRED)
# find_package(Boost COMPONENTS system REQUIRED)
# find_package(PCL COMPONENTS common REQUIRED)


# #find_package(PCL 1.1 REQUIRED)
# #include_directories(${PCL_INCLUDE_DIRS})
# #link_directories(${PCL_LIBRARY_DIRS})
# #add_definitions(${PCL_DEFINITIONS})

# include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})    

# add_executable(pointclouder src/pointcloud_saver.cpp)
# ament_target_dependencies(pointclouder rclcpp std_msgs)

# install(TARGETS
#   pointclouder
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()
cmake_minimum_required(VERSION 3.5)
project(pointcloud_tools)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED QUIET COMPONENTS core features filters io segmentation surface)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Boost COMPONENTS system REQUIRED)
##find_package(PCL COMPONENTS common REQUIRED)

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  EIGEN3
  PCL
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointclouder_saver src/pointcloud_saver.cpp)
ament_target_dependencies(pointclouder_saver
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "EIGEN3"
)
target_link_libraries(pointclouder_saver
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
install(TARGETS pointclouder_saver DESTINATION lib/${PROJECT_NAME})


add_executable(publish src/pointcloud_publisher.cpp)
ament_target_dependencies(publish
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "EIGEN3"
)
target_link_libraries(publish
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
install(TARGETS publish DESTINATION lib/${PROJECT_NAME})

add_executable(pointcloud_selection src/pointcloud_selection.cpp)
ament_target_dependencies(pointcloud_selection
  "pcl_conversions"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "EIGEN3"
)
target_link_libraries(pointcloud_selection
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${PCL_LIBRARIES}
)
install(TARGETS pointcloud_selection DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
