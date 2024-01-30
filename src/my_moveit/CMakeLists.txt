cmake_minimum_required(VERSION 3.8)
project(my_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)

set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "xarm_msgs" "moveit")

# Adding Executables
add_executable(panda_moveit_target_pose src/panda_moveit_target_pose.cpp)
add_executable(xarm6_moveit_target_pose_sub src/xarm6_moveit_target_pose_sub.cpp)
add_executable(xarm6_moveit_target_pose_sub2 src/xarm6_moveit_target_pose_sub2.cpp)
add_executable(xarm6_moveit_target_pose src/xarm6_moveit_target_pose.cpp)
# Include directories for the executables
target_include_directories(panda_moveit_target_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(xarm6_moveit_target_pose_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(xarm6_moveit_target_pose_sub2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(xarm6_moveit_target_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
# Compile features for the executables
target_compile_features(panda_moveit_target_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(xarm6_moveit_target_pose_sub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(xarm6_moveit_target_pose_sub2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(xarm6_moveit_target_pose PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# Dependencies for the executables
ament_target_dependencies(panda_moveit_target_pose
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
ament_target_dependencies(xarm6_moveit_target_pose_sub
  ${dependencies}
)
ament_target_dependencies(xarm6_moveit_target_pose_sub2
  ${dependencies}
)
ament_target_dependencies(xarm6_moveit_target_pose
  ${dependencies}
)

# Install executables
install(TARGETS
  panda_moveit_target_pose
  xarm6_moveit_target_pose_sub
  xarm6_moveit_target_pose_sub2
  xarm6_moveit_target_pose
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
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

ament_package()

