project(samplesink)

add_subdirectory(fileoutput)

if(ENABLE_BLADERF AND LIBBLADERF_FOUND)
    add_subdirectory(bladerf1output)
    add_subdirectory(bladerf2output)
else()
    message(STATUS "Not building bladerf1output or bladerf2output (ENABLE_BLADERF=${ENABLE_BLADERF} LIBBLADERF_FOUND=${LIBBLADERF_FOUND})")
endif()

if(ENABLE_HACKRF AND LIBHACKRF_FOUND)
    add_subdirectory(hackrfoutput)
else()
    message(STATUS "Not building hackrfoutput (ENABLE_HACKRF=${ENABLE_HACKRF} LIBHACKRF_FOUND=${LIBHACKRF_FOUND})")
endif()


if(ENABLE_IIO AND LIBIIO_FOUND)
    add_subdirectory(plutosdroutput)
else()
    message(STATUS "Not building plutosdroutput (ENABLE_IIO=${ENABLE_IIO} LIBIIO_FOUND=${LIBIIO_FOUND})")
endif()
