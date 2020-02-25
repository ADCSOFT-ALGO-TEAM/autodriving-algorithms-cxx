set(LIB_NAME free_space)

find_package(OpenCV  REQUIRED)
aux_source_directory(. DIR_LIB_SRCS)
include_directories(
    ${Opencv_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../include//module/driver_area_2Dto3D/
)

add_library(${LIB_NAME} SHARED ${DIR_LIB_SRCS})
add_library(${LIB_NAME}_static STATIC ${DIR_LIB_SRCS})
set_target_properties(${LIB_NAME}_static PROPERTIES OUTPUT_NAME ${LIB_NAME})
set_target_properties (${LIB_NAME}_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)

set_target_properties (${LIB_NAME} PROPERTIES CLEAN_DIRECT_OUTPUT 1)
set_target_properties (${LIB_NAME} PROPERTIES VERSION 1.0 SOVERSION 1)
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})

file(GLOB INCLUDE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../include//module/driver_area_2Dto3D/*.h)
install (TARGETS ${LIB_NAME} DESTINATION lib)
install (FILES ${INCLUDE_FILES} DESTINATION free_space/include )
