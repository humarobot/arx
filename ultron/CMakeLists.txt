cmake_minimum_required(VERSION 3.0.2)
project(ultron)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs rosbag)
find_package(pinocchio)
find_package(Eigen3)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ultron_node src/main.cpp)
target_link_libraries(ultron_node
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
