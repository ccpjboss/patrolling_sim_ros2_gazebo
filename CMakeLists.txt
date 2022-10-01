cmake_minimum_required(VERSION 3.5)
project(patrolling_sim_ros2_gazebo)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(patrolling_sim_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

# Declare a cpp library
add_library(PatrolAgent 
            src/PatrolAgent.cpp 
            src/getgraph.cpp 
            src/algorithms.cpp
            src/config.cpp)

#target_include_directories(PatrolAgent PUBLIC
#            "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#            "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
#            "$<INSTALL_INTERFACE:include>")      

ament_target_dependencies(PatrolAgent "rclcpp" "ament_index_cpp" 
  "nav2_msgs" "rclcpp_action" "tf2_ros" "nav_msgs" "std_srvs" "tf2_geometry_msgs" "tf2")

#install(
#  TARGETS PatrolAgent EXPORT PatrolAgent
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

# Declare a cpp library
add_library(SSIPatrolAgent 
            src/SSIPatrolAgent.cpp
            src/algorithms.cpp)
            
ament_target_dependencies(SSIPatrolAgent "rclcpp")

include_directories(src)
#ament_export_include_directories(include)
#ament_export_libraries(PatrolAgent)
#ament_export_targets(PatrolAgent)



## Declare a cpp executable
add_executable(Conscientious_Reactive src/Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
##target_link_libraries(Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Conscientious_Reactive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Conscientious_Reactive PatrolAgent)

## Declare a cpp executable
add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive_Agent.cpp)
## Specify libraries to link a library or executable target against
##target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Heuristic_Conscientious_Reactive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Heuristic_Conscientious_Reactive PatrolAgent)
#  
## Declare a cpp executable 
add_executable(Conscientious_Cognitive src/Conscientious_Cognitive_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Conscientious_Cognitive PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Conscientious_Cognitive rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Conscientious_Cognitive PatrolAgent)

## Declare a cpp executable 
add_executable(Cyclic src/Cyclic_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Cyclic PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Cyclic rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(Cyclic PatrolAgent)

## Declare a cpp executable 
add_executable(MSP src/MSP_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(MSP PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(MSP rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(MSP PatrolAgent)

## Declare a cpp executable 
add_executable(GBS src/GBS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(GBS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(GBS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(GBS PatrolAgent)

## Declare a cpp executable 
add_executable(SEBS src/SEBS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(SEBS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(SEBS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(SEBS PatrolAgent)

## Declare a cpp executable 
add_executable(CBLS src/CBLS_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(CBLS PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(CBLS rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs)
target_link_libraries(CBLS PatrolAgent)

## Declare a cpp executable 
add_executable(Random src/Random_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(Random PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(Random rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(Random PatrolAgent)

## Declare a cpp executable 
add_executable(DTAGreedy src/DTAGreedy_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTAGreedy PatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTAGreedy rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTAGreedy PatrolAgent)

## Declare a cpp executable 
add_executable(DTASSI src/DTASSI_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTASSI rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTASSI PatrolAgent SSIPatrolAgent)

## Declare a cpp executable 
add_executable(DTASSIPart src/DTASSIPart_Agent.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent ${catkin_LIBRARIES})
ament_target_dependencies(DTASSIPart rclcpp rclcpp_action nav2_msgs tf2_ros nav_msgs std_msgs)
target_link_libraries(DTASSIPart PatrolAgent SSIPatrolAgent )
 
 
# Results and Monitor Node:
 
## Declare a cpp executable 
add_executable(monitor src/monitor.cpp src/getgraph.cpp)
#add_dependencies(monitor ${PROJECT_NAME}_gencpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(monitor ${catkin_LIBRARIES})
ament_target_dependencies(monitor rclcpp rclcpp_action nav2_msgs tf2_ros std_msgs patrolling_sim_msgs)

add_executable(GoToStartPos src/GoToStartPos.cpp)
#add_dependencies(GoToStartPos ${PROJECT_NAME}_gencpp)
#target_link_libraries(GoToStartPos ${catkin_LIBRARIES})
ament_target_dependencies(GoToStartPos rclcpp rclcpp_action nav2_msgs tf2_ros geometry_msgs patrolling_sim_msgs)


#rosidl_generate_interfaces(${PROJECT_NAME} "srv/GoToStartPosSrv.srv"
  #DEPENDENCIES builtin_interfaces nav2_msgs geometry_msgs rclcpp rclcpp_action)

## Specify libraries to link a library or executable target against
#rosidl_target_interfaces(GoToStartPos ${PROJECT_NAME} "rosidl_typesupport_cpp")


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(patrolling_sim_node patrolling_sim_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION bin
# )

## Mark executables and/or libraries for installation
# install(TARGETS patrolling_sim patrolling_sim_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )
#install(
#  DIRECTORY include/
#  DESTINATION include
#)
## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_patrolling_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(Conscientious_Reactive src/Conscientious_Reactive.cpp)
#add_executable(Heuristic_Conscientious_Reactive src/Heuristic_Conscientious_Reactive.cpp)
#add_executable(Conscientious_Cognitive src/Conscientious_Cognitive.cpp)
#add_executable(Cyclic src/Cyclic.cpp)
#add_executable(MSP src/MSP.cpp)
#add_executable(GBS src/GBS.cpp)
#add_executable(SEBS src/SEBS.cpp)

#set(targets_build 
#Conscientious_Reactive Heuristic_Conscientious_Reactive Conscientious_Cognitive
#Cyclic MSP GBS SEBS CBLS Random DTAGreedy DTASSI DTASSIPart)
 
#install(TARGETS ${targets_build}
# DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp_action)
ament_export_dependencies(nav2_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(stage_ros2)
ament_export_dependencies(std_msgs)
ament_export_dependencies(std_srvs)

# Install launch files.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS monitor Conscientious_Reactive Heuristic_Conscientious_Reactive
  Conscientious_Cognitive Cyclic MSP GBS SEBS CBLS Random DTAGreedy
  DTASSI DTASSIPart GoToStartPos DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY description DESTINATION share/${PROJECT_NAME})
install(DIRECTORY MSP DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION $ENV{HOME}/.gazebo/models/${PROJECT_NAME})

ament_package()