cmake_minimum_required(VERSION 2.8)
project(rdda C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread rt m)

# SOEM
#include_directories(/usr/include/soem)
find_package(soem)
#include_directories(../SOEM/build)
#include_directories(../SOEM/soem)
#include_directories(../SOEM/osal)
#include_directories(../SOEM/osal/linux)
#include_directories(../SOEM/oshw/linux)

#link_directories(/home/steph/SOEM/soem/)
#add_library(SOEM_LIB STATIC /home/steph/SOEM/build/libsoem.a)
#add_library(SOEM_LIB STATIC /home/steph/SOEM/soem/ethercat.h)
#set_target_properties(SOEM_LIB PROPERTIES LINKER_LANGUAGE C)



#FIND_LIBRARY(SOEM_LIB /home/step/SOEM/build)
#link_directories(SOEM_LIB)
#link_directories(/home/step/SOEM/build)

# Add the headers from the <include> directory
include_directories(
        include/${PROJECT_NAME}/
        include/shm
        include/toolbox
)

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_master_lib
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

### Build RDDA slave
add_library(${PROJECT_NAME}_slave_lib
        src/${PROJECT_NAME}/slave/rdda_ecat.c
        src/${PROJECT_NAME}/common/init_BEL.c
        src/${PROJECT_NAME}/slave/rdda_base.c
        src/${PROJECT_NAME}/slave/rdda_control.c
        src/toolbox/contact_detect.c
        src/shm/shm.c
        src/toolbox/slave/tele_control.c)

target_link_libraries(${PROJECT_NAME}_slave_lib soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_slave
        src/${PROJECT_NAME}/slave/rdda_simpletest.c
        )

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_slave
        ${PROJECT_NAME}_slave_lib
        )
