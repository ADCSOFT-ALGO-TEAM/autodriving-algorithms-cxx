cmake_minimum_required(VERSION  2.8)
project( convert_2D_to_3D)
# Set C/C++ flags

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Werror  -lglog -lgflags -pthread -fPIE -fPIC -Wno-attributes -DQT_NO_VERSION_TAGGING")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -std=c11 -fPIE -fPIC -Wno-attributes -DQT_NO_VERSION_TAGGING")

find_package(OpenCV  REQUIRED)
set(CMAKE_MACOSX_RPATH 1)
#set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/install)
#set(CMAKE_INSTALL_PREFIX /opt/adc/)
set(CMAKE_INSTALL_PREFIX /opt/adc/)

add_subdirectory(src/module/driver_area_2Dto3D free_space)
#for driver area test
###############################################
set(APP_DRIVER_EARA src/app/test_driver_area_2Dto3D.cpp)

include_directories(
    ${Opencv_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include//module/driver_area_2Dto3D/
)
add_executable(test_driver_area_2Dto3D_app  ${APP_DRIVER_EARA})
target_link_libraries(test_driver_area_2Dto3D_app ${OpenCV_LIBS} free_space)
##############################################


# #for driver earea calibration app
# ##############################################
set(APP_DRIVER_EARA_CALIBRATION src/app/calibration_driver_area.cpp)

include_directories(
    ${Opencv_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include//module/driver_area_2Dto3D/
    )
add_executable(calibration_driver_area_app ${APP_DRIVER_EARA_CALIBRATION})
target_link_libraries(calibration_driver_area_app ${OpenCV_LIBS} free_space)
# ##############################################

install(TARGETS test_driver_area_2Dto3D_app DESTINATION bin )
install(TARGETS calibration_driver_area_app DESTINATION bin )

