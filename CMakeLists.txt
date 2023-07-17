cmake_minimum_required(VERSION 3.8)
project(travel)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io visualization filters)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions  REQUIRED)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_find_build_dependencies()

include_directories(
        include
        SYSTEM
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

set(travel_src
        src/travel_node.cpp
        )

ament_auto_add_library(travel SHARED
        ${travel_src}
        )

if(${PCL_VERSION} GREATER_EQUAL 1.12.1)
    find_package(Qhull REQUIRED)
    target_link_libraries(travel
            QHULL::QHULL
            )
endif()

target_link_libraries(travel
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        )

rclcpp_components_register_node(travel
        PLUGIN "travel::Travel"
        EXECUTABLE travel_exe
        )

ament_auto_package()
