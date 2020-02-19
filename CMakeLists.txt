cmake_minimum_required(VERSION  2.8)
project( convert_2D_to_3D)
# Set C/C++ flags

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Werror  -lglog -lgflags -pthread -fPIE -fPIC -Wno-attributes -DQT_NO_VERSION_TAGGING")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -std=c11 -fPIE -fPIC -Wno-attributes -DQT_NO_VERSION_TAGGING")

#for driver area test
###############################################
aux_source_directory(src/module/driver_area_2Dto3D SRC_LIST)
set(APP_DRIVER_EARA src/app/test_driver_area_2Dto3D.cpp)

find_package(OpenCV  REQUIRED)

include_directories(
    ${Opencv_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include//module/driver_area_2Dto3D/
)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${APP_DRIVER_EARA})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
###########################################

