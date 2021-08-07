cmake_minimum_required(VERSION 3.0.2)
project(rdda)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
# set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread rt m)

# catkin
find_package(catkin REQUIRED)

# SOEM
find_package(soem REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rdda_ros
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include/${PROJECT_NAME}/
        include/shm
        include/toolbox
)

add_library(${PROJECT_NAME}_master_lib STATIC
        src/${PROJECT_NAME}/master/rdda_ecat.c
        src/${PROJECT_NAME}/common/init_BEL.c
        src/${PROJECT_NAME}/master/rdda_base.c
        src/${PROJECT_NAME}/master/rdda_control.c
        src/toolbox/contact_detect.c
        src/shm/shm.c
        src/toolbox/master/tele_control.c)

target_link_libraries(${PROJECT_NAME}_master_lib soem ${OS_LIBS})
# Add the executable files
add_executable(${PROJECT_NAME}_master
        src/${PROJECT_NAME}/master/rdda_simpletest.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_master
        ${PROJECT_NAME}_master_lib
        )

add_library(${PROJECT_NAME}_slave_lib STATIC
        src/${PROJECT_NAME}/slave/rdda_ecat.c
        src/${PROJECT_NAME}/common/init_BEL.c
        src/${PROJECT_NAME}/slave/rdda_base.c
        src/${PROJECT_NAME}/slave/rdda_control.c
        src/toolbox/contact_detect.c
        src/shm/shm.c
        src/toolbox/slave/tele_control.c
        )

target_link_libraries(${PROJECT_NAME}_slave_lib soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_slave
        src/${PROJECT_NAME}/slave/rdda_simpletest.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_slave
        ${PROJECT_NAME}_slave_lib
        )
