cmake_minimum_required(VERSION 2.8)
project(rdda C)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
# set(OS_LIBS pthread rt)

# SOEM
# find_package(SOEM REQUIRED)
include_directories(/home/ethercat-master/SOEM-master/soem)
include_directories(/home/ethercat-master/SOEM-master/osal)
include_directories(/home/ethercat-master/SOEM-master/osal/linux)
include_directories(/home/ethercat-master/SOEM-master/oshw/linux)
# link_directories(/home/ethercat-master/SOEM-master/soem)


# Add the headers from the <include> directory
include_directories(include)
include_directories(src/${PROJECT_NAME}/)

# Add the source files from the <src> directory
add_library(${PROJECT_NAME}_rddamain src/${PROJECT_NAME}/rddamain.c src/rdda/init_BEL.c include/init_BEL.h)

# Linking for libraries
target_link_libraries(${PROJECT_NAME}_rddamain)

