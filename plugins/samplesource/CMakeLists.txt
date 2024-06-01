project(samplesource)

add_subdirectory(fileinput)

if(ENABLE_BLADERF AND LIBBLADERF_FOUND)
    add_subdirectory(bladerf1input)
    add_subdirectory(bladerf2input)
else()
    message(STATUS "Not building bladerf1input or bladerf2input (ENABLE_BLADERF=${ENABLE_BLADERF} LIBBLADERF_FOUND=${LIBBLADERF_FOUND})")
endif()

if(ENABLE_HACKRF AND LIBHACKRF_FOUND)
    add_subdirectory(hackrfinput)
else()
    message(STATUS "Not building hackrfinput (ENABLE_HACKRF=${ENABLE_HACKRF} LIBHACKRF_FOUND=${LIBHACKRF_FOUND})")
endif()


if(ENABLE_IIO AND LIBIIO_FOUND)
    add_subdirectory(plutosdrinput)
else()
    message(STATUS "Not building plutosdrinput (ENABLE_IIO=${ENABLE_IIO} LIBIIO_FOUND=${LIBIIO_FOUND})")
endif()