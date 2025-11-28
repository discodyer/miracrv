cmake_minimum_required(VERSION 3.8)
project(miracrv_driver)
if(POLICY CMP0148) 
cmake_policy(SET CMP0148 OLD) 
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## UTF-8 编码
add_compile_options(-fexec-charset=UTF-8)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 获取 src/lib/ 目录下的所有源文件
file(GLOB RACER_SOURCES "src/libracer/*.cpp")

# 将获取的源文件添加到库中
add_library(${PROJECT_NAME} SHARED ${RACER_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# include_directories(
#   include
#   ${CMAKE_CURRENT_BINARY_DIR}/include  # 添加生成的包含目录
# )

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  geographic_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(DIRECTORY include/
#   DESTINATION include
# )

# set(msg_files
#   "msg/TargetObject.msg"
# )

# # 添加自定义消息
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )
# ament_export_dependencies(rosidl_default_runtime)

# Add executable
add_executable(ardu_rover_node src/arduRoverNode.cpp)
target_link_libraries(ardu_rover_node ${PROJECT_NAME})

# Install the executable
install(TARGETS ardu_rover_node
  DESTINATION lib/${PROJECT_NAME}
)

# rosidl_get_typesupport_target(cpp_typesupport_target
#   ${PROJECT_NAME} rosidl_typesupport_cpp)

# target_link_libraries(racer "${cpp_typesupport_target}")

# see here: https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#link-against-the-interface

install(PROGRAMS
  scripts/tf_to_odom.py
  scripts/odom_to_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  geographic_msgs
  tf2_ros
  tf2_geometry_msgs
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
