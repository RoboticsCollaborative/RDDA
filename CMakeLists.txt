cmake_minimum_required(VERSION 2.8)
project(rdda C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread rt m)

# SOEM
find_package(soem)

# Add the headers from the <include> directory
include_directories(
        include/${PROJECT_NAME}/
        include/shm
        include/toolbox
)

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_master_lib
        src/${PROJECT_NAME}/rdda_ecat.c
        src/${PROJECT_NAME}/init_AEV.c
        src/${PROJECT_NAME}/rdda_base.c
        src/${PROJECT_NAME}/rdda_control.c
        src/shm/shm.c
        src/toolbox/master/tele_control.c)

target_link_libraries(${PROJECT_NAME}_master_lib soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_master
        src/${PROJECT_NAME}/rdda_simpletest.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_master
        ${PROJECT_NAME}_master_lib
        )

### Build RDDA slave
add_library(${PROJECT_NAME}_slave_lib
        src/${PROJECT_NAME}/rdda_ecat.c
        src/${PROJECT_NAME}/init_AEV.c
        src/${PROJECT_NAME}/rdda_base.c
        src/${PROJECT_NAME}/rdda_control.c
        src/shm/shm.c
        src/toolbox/slave/tele_control.c
        )

target_link_libraries(${PROJECT_NAME}_slave_lib soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_slave
        src/${PROJECT_NAME}/rdda_simpletest.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_slave
        ${PROJECT_NAME}_slave_lib
        )
