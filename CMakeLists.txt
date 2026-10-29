cmake_minimum_required(VERSION 3.16)
project(pb_omni_pid_pursuit_controller)

find_package(ament_cmake REQUIRED)
set(dependencies rclcpp map_server geometry_msgs nav_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/omni_pid_pursuit_controller.cpp src/pid.cpp src/collision_check.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config integration DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller ${PROJECT_NAME})
  ament_target_dependencies(test_controller ${dependencies})
  set_tests_properties(test_controller PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=193;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_test_logs")
endif()
ament_package()
