cmake_minimum_required(VERSION 3.8)
project(kiwi_vff_avoidance)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/kiwi_vff_avoidance/AvoidanceNode.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(avoidance_vff src/avoidance_vff_main.cpp)
ament_target_dependencies(avoidance_vff ${dependencies})
target_link_libraries(avoidance_vff ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  avoidance_vff
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_dependencies(${dependencies})
ament_package()
