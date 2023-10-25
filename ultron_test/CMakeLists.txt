cmake_minimum_required(VERSION 3.0.2)
project(ultron_test)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs 
  sensor_msgs 
  rosbag 
  arm_control
  serial
  )
find_package(pinocchio)
find_package(Eigen3)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(ultron_position_node src/main_position.cpp)
target_link_libraries(ultron_position_node
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(ultron_position_node PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron.urdf")


add_executable(ultron_impedende_node src/main_impedence.cpp)
target_link_libraries(ultron_impedende_node
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(ultron_impedende_node PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron.urdf")

add_executable(ultron_traj_node src/main_traj.cpp)
target_link_libraries(ultron_traj_node
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(ultron_traj_node PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron.urdf")

add_executable(ultron_zero_pos src/zero_pos.cpp)
target_link_libraries(ultron_zero_pos
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(ultron_zero_pos PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron.urdf")

add_executable(pos_impedence src/pos_impedence.cpp)
target_link_libraries(pos_impedence
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(pos_impedence PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron3.urdf")

add_executable(demonstrate_mode 
src/demonstrate_mode.cpp
src/demonstrator.cpp)

target_link_libraries(demonstrate_mode
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
target_compile_definitions(demonstrate_mode PRIVATE URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/urdf/ultron3.urdf")

add_executable(hand_controller
src/hand_controller.cpp)

target_link_libraries(hand_controller
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
