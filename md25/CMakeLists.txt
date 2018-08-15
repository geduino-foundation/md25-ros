# md25/CMakeList.txt
#
# Copyright (C) 2015 Alessandro Francescon
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 2.8.3)

project(md25)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    diagnostic_msgs
    sensor_msgs
    md25_msgs
    dynamic_reconfigure
    tf
    serial
)

find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/MD25Controller.cfg
    cfg/MD25Odometry.cfg
)

catkin_package(
    DEPENDS Eigen
    CATKIN_DEPENDS message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    src
)

add_executable(md25_console src/md25_console.cpp src/md25.cpp)

add_executable(md25_driver_node src/md25_driver_node.cpp src/md25_driver.cpp src/md25.cpp src/asleep_rate.cpp)
add_dependencies(md25_driver_node ${catkin_EXPORTED_TARGETS})

add_executable(md25_odometry_node src/md25_odometry_node.cpp src/md25_odometry.cpp src/ddr_odometry.cpp src/yaw_filter.cpp src/rolling_window.cpp)
add_dependencies(md25_odometry_node md25_odometry_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(md25_controller_node src/md25_controller_node.cpp src/md25_controller.cpp src/pid.cpp)
add_dependencies(md25_controller_node md25_controller_gencfg ${catkin_EXPORTED_TARGETS})

add_executable(md25_simple_controller_node src/md25_simple_controller_node.cpp src/md25_simple_controller.cpp)
add_dependencies(md25_simple_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(odom_to_euler_node src/odom_to_euler_node.cpp)
add_dependencies(odom_to_euler_node ${catkin_EXPORTED_TARGETS})

add_library(md25_driver_nodelet src/md25_driver_nodelet.cpp src/md25_driver.cpp src/md25.cpp src/asleep_rate.cpp)
add_dependencies(md25_driver_nodelet ${catkin_EXPORTED_TARGETS})

add_library(md25_odometry_nodelet src/md25_odometry_nodelet.cpp src/md25_odometry.cpp src/ddr_odometry.cpp src/yaw_filter.cpp src/rolling_window.cpp)
add_dependencies(md25_odometry_nodelet md25_odometry_gencfg ${catkin_EXPORTED_TARGETS})

add_library(md25_controller_nodelet src/md25_controller_nodelet.cpp src/md25_controller.cpp src/pid.cpp)
add_dependencies(md25_controller_nodelet md25_controller_gencfg ${catkin_EXPORTED_TARGETS})

add_library(md25_simple_controller_nodelet src/md25_simple_controller_nodelet.cpp src/md25_simple_controller.cpp)
add_dependencies(md25_simple_controller_nodelet ${catkin_EXPORTED_TARGETS})

target_link_libraries(md25_console
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_driver_node
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_odometry_node
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_simple_controller_node
  ${catkin_LIBRARIES}
)

target_link_libraries(odom_to_euler_node
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_driver_nodelet
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_odometry_nodelet
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_controller_nodelet
  ${catkin_LIBRARIES}
)

target_link_libraries(md25_simple_controller_nodelet
  ${catkin_LIBRARIES}
)
