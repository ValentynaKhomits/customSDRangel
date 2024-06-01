project(devices)

include_directories(
    # export.h
    ${CMAKE_SOURCE_DIR}/exports
    # util/message.h
    ${CMAKE_SOURCE_DIR}/sdrbase
)

if(ENABLE_BLADERF AND LIBBLADERF_FOUND)
    add_subdirectory(bladerf1)
    add_subdirectory(bladerf2)
endif()

if(ENABLE_HACKRF AND LIBHACKRF_FOUND)
    add_subdirectory(hackrf)
endif()
