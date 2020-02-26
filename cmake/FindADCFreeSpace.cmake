SET(LIBRARY_NAMES free_space)
FIND_PATH(ADC_FREE_SPACE_INCLUDE driver_area_2Dto3D.h /usr/free_space/  /usr/free_space/include/ /usr/include /usr/include/adc_free_space /usr/local/include /usr/local/include/adc_free_space /opt/adc/include/free_space)
FIND_LIBRARY(FREE_SPACE_LIBRARY NAMES ${LIBRARY_NAMES}  /usr/lib /usr/lib/adc_free_space /usr/local/lib /usr/local/lib/adc_free_space /opt/adc/lib)

IF(ADC_FREE_SPACE_INCLUDE AND FREE_SPACE_LIBRARY)
    SET(ADC_FREE_SPACE_FOUND TRUE)
ENDIF(ADC_FREE_SPACE_INCLUDE AND FREE_SPACE_LIBRARY)

IF(ADC_FREE_SPACE_FOUND)
    IF(NOT ADC_FREE_SPACE_FOUND_QUIETLY)
        MESSAGE(STATUS "Found ADC_FREE_SPACE: " ${FREE_SPACE_LIBRARY})
    ENDIF(NOT ADC_FREE_SPACE_FOUND_QUIETLY)
ELSEIF(ADC_FREE_SPACE_FOUND)
    IF(ADC_FREE_SPACE_FOUND_REQUIRED)
        MESSAGE(FATAL_ERROR "Could not find ADC_FREE_SPACE libarary")
    ENDIF(ADC_FREE_SPACE_FOUND_REQUIRED)
ENDIF(ADC_FREE_SPACE_FOUND)