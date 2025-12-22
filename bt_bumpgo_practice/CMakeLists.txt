cmake_minimum_required(VERSION 3.8)
project(bt_bumpgo_practice)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
  message(STATUS "ZMQ found.")
  add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZMQ not found.")
endif()

set(dependencies
  rclcpp
  behaviortree_cpp_v3
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(br2_forward_bt_node SHARED IMPORTED)
set_property(TARGET br2_forward_bt_node PROPERTY IMPORTED_LOCATION
  "/home/kiwi101117/bookros2_ws/install/br2_bt_bumpgo/lib/libbr2_forward_bt_node.so"
)
target_include_directories(br2_forward_bt_node INTERFACE 
  "/home/kiwi101117/bookros2_ws/install/br2_bt_bumpgo/include"
)

add_executable(practice_forward src/practice_forward.cpp)
ament_target_dependencies(practice_forward ${dependencies})
target_link_libraries(practice_forward ${ZMQ_LIBRARIES})

install(TARGETS
  practice_forward
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
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
