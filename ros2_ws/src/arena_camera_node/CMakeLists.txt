cmake_minimum_required(VERSION 3.15)

project(arena_camera_node)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# linked GenICam hearders give warnings No such file or directory
	# GENICAM_USER_ACCEPTS_ANY_COMPILER flag need tro be defined 
	# 
	#add_compile_options(-Wall -Wextra -Wpedantic -Wunknown-pragmas)
endif()

# -----------------------------------------------------------------------------
#
# ALL PKGS SETTIGNS
#
# -----------------------------------------------------------------------------


# ------------------
# CUSTOM DEPS PATHS
# ------------------

# arenasdk find file dir
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# ----------
# FIND DEPS
# ----------

# finds dependencies
# - same as the ones mentioned in package.xml under
#   the tags <buildtool_depend>, <build_depend>, <exec_depend>
# - for pakages used in #includes
# - even if a dependency is used in one package, it has to be found
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arena_sdk REQUIRED)
find_package(OpenCV REQUIRED)     # 🔹 추가
find_package(cv_bridge REQUIRED)  # 🔹 추가

# ------------
# EXPORT DEPS
# ------------

# export dependencies
# For packages that depends on arena_camera_node
# basically the dependecies of the depencey
# these are the one mentioned in
# - "find_package"
# - "ament_target_dependencies"
# - forgetting to export a depencey would not show at the time of
#   building the package but when other packages use this package
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(std_msgs)
# Export package dependencies
#ament_export_dependencies(ament_cmake)
#ament_export_dependencies(ament_cmake_python)
#ament_export_dependencies(rclcpp)
#ament_export_dependencies(rclpy)
#ament_export_dependencies(sensor_msgs)
#ament_export_dependencies(std_srvs)

# -------------
# LAUNCH FILES
# -------------

# install launch files.
#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}/
# )

# --------
# HEADERS
# --------

# MV .hpp (s) to include dir
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION include/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.hpp")

#
# EXPORT HEADERS should it before the "ament_export_dependencies" ?
#
#ament_export_include_directories(include)
#ament_export_include_directories(include ${rclcpp_INCLUDE_DIRS})


# -----------------------------------------------------------------------------
#
# ROS2 RUN ARENA_CAMERA_NODE <>
#
# -----------------------------------------------------------------------------
set(polar_aolp_dolp polar_aolp_dolp)

# CPP -> EXECUTABLE
add_executable(${polar_aolp_dolp}
    src/polarization_aolp_dolp.cpp
)

# ROS2 DEPENDENCIES
ament_target_dependencies(${polar_aolp_dolp}
    rclcpp
    sensor_msgs
    cv_bridge
    arena_sdk
)

# INCLUDE DIRS (OpenCV 등 외부 경로 필요시 명시)
target_include_directories(${polar_aolp_dolp}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)

# LINK OpenCV (필수)
target_link_libraries(${polar_aolp_dolp} ${OpenCV_LIBS})

# INSTALL EXECUTABLE
install(TARGETS
  ${polar_aolp_dolp}
  DESTINATION lib/${PROJECT_NAME}
)

set(polar_all polar_all)

# CPP -> EXECUTABLE
add_executable(${polar_all}
    src/polarization_all.cpp
)

# ROS2 DEPENDENCIES
ament_target_dependencies(${polar_all}
    rclcpp
    sensor_msgs
    cv_bridge
    arena_sdk
)

# INCLUDE DIRS (OpenCV 등 외부 경로 필요시 명시)
target_include_directories(${polar_all}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>
)

# LINK OpenCV (필수)
target_link_libraries(${polar_all} ${OpenCV_LIBS})

# INSTALL EXECUTABLE
install(TARGETS
  ${polar_all}
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
set(target_name start)

# CPP DEPS
add_executable(${target_name}
    src/main.cpp
    src/ArenaCameraNode.cpp
    src/light_arena/deviceinfo_helper.cpp
    src/rclcpp_adapter/pixelformat_translation.cpp
    src/rclcpp_adapter/quilty_of_service_translation.cpp
)

# HEADERS DEP
#target_include_directories(${target_name}
  #PRIVATE
    #${arena_sdk_INCLUDE_DIRS}
#  PUBLIC
    #$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    #$<INSTALL_INTERFACE:include>
#)

# CPP -> .a file
# creates :
# - build/arena_camera_node/CMakeFiles/arena_camera_node.dir and sub dirs
# - build/arena_camera_node/libarena_camera_node.a
# add_library(${PROJECT_NAME} src/main.cpp)

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# ROS DEP
# - this way, it will find our header libs automatically
# - for Arenasdk the cmake way is used no ament
ament_target_dependencies(${target_name}  rclcpp 
                                          std_msgs 
                                          std_srvs
                                          sensor_msgs
                                          arena_sdk
                                          )


# MV EXEC to lib dir
install(TARGETS
  ${target_name} 
    DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
#
# ROS2 RUN TRIGGER_IMAGE <>
#
# -----------------------------------------------------------------------------

set(trigger_image trigger_image)

# CPP -> EXECUTABLE
add_executable(${trigger_image} src/trigger_image_client.cpp)

# CPP -> .a file

# EXPORT .a file for other pakges ?
#ament_export_libraries(${PROJECT_NAME} ${rclcpp_LIBRARIES})

# ROS DEP
# - this way it will find our header libs automatically
# - for ArenAsdk the cmake way is used no ament
ament_target_dependencies(${trigger_image}  rclcpp 
                                          std_srvs)

# HEADERS DEP
target_include_directories(${trigger_image}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# MV EXEC to lib dir
install(TARGETS
  ${trigger_image} 
    DESTINATION lib/${PROJECT_NAME}
)
# -----------------------------------------------------------------------------

#
#
# PYTHON
#
#

#ament_cmake_python()


# invistifate ----------------------------
# Install python scripts
#install(DIRECTORY arena_camera_node
#        DESTINATION lib
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")

#install(DIRECTORY arena_camera_node
#        DESTINATION ../build
#        FILES_MATCHING REGEX "arena_camera_node/.+\.py")
# TESTING ---------------------------------------------------------------------

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

ament_package()

#message(FATAL_ERROR ********************************************)