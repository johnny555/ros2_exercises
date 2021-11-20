cmake_minimum_required(VERSION 3.5)
project(cpp_unit_7_services)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
include_directories(include)

add_executable(empty_service_server src/empty_service_server.cpp)
add_executable(cpp_simple_service_client src/cpp_simple_service_client.cpp)
add_executable(square_server src/square_server.cpp)

add_library(move_bb8 SHARED
	src/move_bb8.cpp
)


ament_export_libraries(move_bb8)


ament_target_dependencies(cpp_simple_service_client rclcpp std_msgs gazebo_msgs)
ament_target_dependencies(empty_service_server rclcpp std_msgs std_srvs)
ament_target_dependencies(square_server 
gazebo_msgs rclcpp std_msgs std_srvs geometry_msgs )

ament_target_dependencies(move_bb8 rclcpp std_msgs std_srvs geometry_msgs)

target_link_libraries(square_server move_bb8 std_msgs std_srvs geometry_msgs)

install(TARGETS
   cpp_simple_service_client
   empty_service_server
   square_server
   DESTINATION lib/${PROJECT_NAME}
 )

 # Install launch files.
install(DIRECTORY
 launch
 DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
        DESTINATION include)


ament_package()
