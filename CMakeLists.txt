cmake_minimum_required(VERSION 2.8)
project(rdda C CXX)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_POSITION_INDEPENDENT_CODE true)
set(OS_LIBS pthread rt)

# SOEM
include_directories(/usr/include/soem)
#find_package(soem)
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
include_directories(include/${PROJECT_NAME}/ include/shm)
include_directories(src/${PROJECT_NAME}/ src/shm)

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_src
        src/${PROJECT_NAME}/rdda_ecat.c
        src/${PROJECT_NAME}/init_BEL.c
        src/${PROJECT_NAME}/rdda_base.c
        src/shm/shm.c
#        src/shm/shm_master.c
#        src/shm/shm_slave.c
        )

target_link_libraries(${PROJECT_NAME}_src soem ${OS_LIBS})

# Add the executable files
add_executable(${PROJECT_NAME}_simpletest src/${PROJECT_NAME}/rdda_simpletest.c)
# Linking for libraries
target_link_libraries(${PROJECT_NAME}_simpletest
        ${PROJECT_NAME}_src
        soem
        ${OS_LIBS}
        )

