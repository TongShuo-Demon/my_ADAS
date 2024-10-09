# project name
PROJECT(ADAS)
# requirement of cmake version
cmake_minimum_required(VERSION 3.5)
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# set the build type to debug
set(CMAKE_BUILD_TYPE DEBUG)
add_definitions(-g)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${ADAS_SOURCE_DIR}/bin)
message("start to use ADAS")

# find required opencv
find_package(OpenCV REQUIRED)
# directory of opencv headers
include_directories(${OpenCV_INCLUDE_DIRS})
# 查找Eigen3 library
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# yaml-cpp library
find_package(yaml-cpp REQUIRED)
include_directories(/usr/local/include/yaml-cpp)

find_package(Protobuf REQUIRED)
include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
)


include_directories( ${PROJECT_SOURCE_DIR}/2d_avm/include
                     ${PROJECT_SOURCE_DIR}/utilities/include
                     ${PROJECT_SOURCE_DIR}/config
                     ${PROJECT_SOURCE_DIR}/single_view/include
                     ${PROJECT_SOURCE_DIR}/transparent_chassis/include
           
                     )

add_library(addressbook_protobuf config/avm2d_config.pb.cc)
# add_library(test_protobuf config/example.pb.cc)

# name of executable file and path of source file
add_executable(ADAS main.cpp 2d_avm/sources/fisheye_camera_projective.cpp  
                            2d_avm/sources/birdview.cpp
                            2d_avm/sources/2d_avm.cpp
                            2d_avm/sources/data_config.cpp
                            utilities/sources/image_processing.cpp
                            ) 
#single view
add_executable(single_view single_view_main.cpp single_view/sources/single_view.cpp  )
target_link_libraries(single_view addressbook_protobuf ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PROTOBUF_LIBRARIES})

#test view
# add_executable(test main_.cpp   )
# target_link_libraries(test test_protobuf ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PROTOBUF_LIBRARIES})

# 透明底盘
add_executable(transparent_chassis transparent_chassis_main.cpp transparent_chassis/sources/transparent_chassis_cpu_api.cpp  
                                              transparent_chassis/sources/transparent_chassis_cpu.cpp)
target_link_libraries(transparent_chassis addressbook_protobuf ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PROTOBUF_LIBRARIES})


link_directories(${OpenCV_LIBRARY_DIRS}  )
# opencv libraries
target_link_libraries(ADAS addressbook_protobuf ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} ${PROTOBUF_LIBRARIES})


