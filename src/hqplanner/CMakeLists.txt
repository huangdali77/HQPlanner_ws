cmake_minimum_required(VERSION 3.0.2)
project(hqplanner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  cmake_modules
  visualization_msgs
  tf
)
find_package(Eigen REQUIRED) 
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
add_message_files(
  FILES
  ref_line_test.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)
################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hqplanner
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime visualization_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${Eigen_INCLUDE_DIRS})
## Declare a C++ library
add_library(hqplanner
  src/common/frame.cpp
  src/common/obstacle.cpp
  src/common/path_decision.cpp
  src/common/path_obstacle.cpp

  src/for_proto/config_param.cpp
  src/for_proto/vehicle_config_helper.cpp
  src/for_proto/vehicle_state_provider.cpp
     
  src/main/anchor_points_provider.cpp
  src/main/environment_publish.cpp
  src/main/global_number_provider.cpp
  src/main/planning.cpp
  src/main/prediction_obstacles_provider.cpp
  src/main/manual_setting.cpp

  src/math/aabox2d.cpp
  src/math/box2d.cpp
  src/math/cartesian_frenet_conversion.cpp
  src/math/cubic_spline.cpp
  src/math/cubic_spline_clamped.cpp
  src/math/cubic_spline_start_clamped.cpp
  src/math/line_segment2d.cpp
  src/math/linear_interpolation.cpp
  src/math/math_utils.cpp
  src/math/polygon2d.cpp
  src/math/vec2d.cpp
  src/math/curve1d/quartic_polynomial_curve1d.cpp
  src/math/curve1d/quintic_polynomial_curve1d.cpp

  src/path/discretized_path.cpp
  src/path/frenet_frame_path.cpp
  src/path/path_data.cpp

  src/reference_line/reference_line.cpp
  src/reference_line/reference_line_info.cpp
  src/reference_line/reference_line_provider.cpp

  src/speed/speed_data.cpp
  src/speed/speed_limit.cpp
  src/speed/st_boundary.cpp
  src/speed/st_point.cpp

  src/tasks/path_optimizer.cpp
  src/tasks/speed_optimizer.cpp
  src/tasks/task.cpp
  src/tasks/dp_poly_path/dp_poly_path_optimizer.cpp
  src/tasks/dp_poly_path/dp_road_graph.cpp
  src/tasks/dp_poly_path/trajectory_cost.cpp
  src/tasks/dp_st_speed/dp_st_cost.cpp
  src/tasks/dp_st_speed/dp_st_graph.cpp
  src/tasks/dp_st_speed/dp_st_speed_optimizer.cpp
  src/tasks/dp_st_speed/st_graph_point.cpp
  src/tasks/em/em_planner.cpp
  src/tasks/path_decider/path_decider.cpp
  src/tasks/speed_decider/speed_decider.cpp
  src/tasks/st_graph/speed_limit_decider.cpp
  src/tasks/st_graph/st_boundary_mapper.cpp
  src/tasks/st_graph/st_graph_data.cpp

  src/trajectory/discretized_trajectory.cpp
  src/trajectory/publishable_trajectory.cpp
  src/trajectory/trajectory_stitcher.cpp

  src/util/util.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hqplanner_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(ref_line_pub src/reference_line_test.cpp)
#add_dependencies(ref_line_pub ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(ref_line_pub hqplanner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(ref_line_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${catkin_LIBRARIES})
target_link_libraries(ref_line_pub hqplanner ${catkin_LIBRARIES})

add_executable(main src/main/main.cpp)
add_dependencies(main hqplanner ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(main hqplanner ${catkin_LIBRARIES})


add_executable(ros_test src/ros_test.cpp)
target_link_libraries(ros_test ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hqplanner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
