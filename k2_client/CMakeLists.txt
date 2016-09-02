cmake_minimum_required(VERSION 2.8.3)
project(k2_client)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(YAMLCPP_STATIC_LIBRARY ON CACHE BOOL
    "If true, try to find static yaml-cpp library first instead of dynamic.")
find_package(YamlCpp REQUIRED)
include_directories(${YAMLCPP_INCLUDE_DIRS})
add_definitions(${YAMLCPP_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    camera_info_manager
    message_generation
)

add_message_files(FILES
    Audio.msg    
    BodyArray.msg
    JointOrientationAndType.msg
    JointPositionAndState.msg
    Lean.msg
    Body.msg
    Face.msg
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        std_msgs
        message_runtime
    DEPENDS
        yaml-cpp
)

# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")

# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

# Enable C++11.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -Wno-unused-parameter")

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${catkin_LIBRARY_DIRS}
)

add_executable(audio  "${PROJECT_SOURCE_DIR}/src/audio.cpp")
add_executable(bodies "${PROJECT_SOURCE_DIR}/src/bodies.cpp")
add_executable(depth  "${PROJECT_SOURCE_DIR}/src/depth.cpp")
add_executable(ir     "${PROJECT_SOURCE_DIR}/src/ir.cpp")
add_executable(rgb    "${PROJECT_SOURCE_DIR}/src/rgb.cpp")
add_executable(faces  "${PROJECT_SOURCE_DIR}/src/faces.cpp")

add_dependencies(audio  ${k2_client_EXPORTED_TARGETS})
add_dependencies(bodies ${k2_client_EXPORTED_TARGETS})
add_dependencies(faces  ${k2_client_EXPORTED_TARGETS})
add_dependencies(depth  ${k2_client_EXPORTED_TARGETS})
add_dependencies(ir     ${k2_client_EXPORTED_TARGETS})
add_dependencies(rgb    ${k2_client_EXPORTED_TARGETS})

target_link_libraries(audio  ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(bodies ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(faces  ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(depth  ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(ir     ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(rgb    ${YAMLCPP_LIBRARIES} ${catkin_LIBRARIES})



