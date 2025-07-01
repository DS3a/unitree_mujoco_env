cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)


# Find Pinocchio using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# Include directories and link libraries
include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})
add_definitions(${PINOCCHIO_CFLAGS_OTHER})

# add_subdirectory(../osqp osqp-build)
# add_subdirectory(../QuadProgpp quadprogpp-build)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/whole_body_roller)
# find_package(whole_body_roller REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../whole_body_roller)

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
    src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2)


add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco ${SIM_DEPENDENCIES})

add_executable(test test/test_unitree_sdk2.cpp)
target_link_libraries(test unitree_sdk2)

add_executable(stand_go2 stand/stand_go2.cpp)
target_link_libraries(stand_go2 unitree_sdk2)

add_executable(stand_h1 stand/stand_h1.cpp)
target_link_libraries(stand_h1 quadprogpp)
target_link_libraries(stand_h1  unitree_sdk2)
target_link_libraries(stand_h1 whole_body_roller)
target_link_libraries(stand_h1 ${PINOCCHIO_LIBRARIES})

add_executable(pd_controller pd_controller/pd_controller.cpp)
target_link_libraries(pd_controller  unitree_sdk2)
target_link_libraries(pd_controller ${PINOCCHIO_LIBRARIES})


add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

set(CMAKE_BUILD_TYPE Release)
