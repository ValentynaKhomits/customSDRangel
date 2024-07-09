project(demod)

if (ENABLE_CHANNELRX_DEMODATV)
    add_subdirectory(demodatv)
else()
    message(STATUS "Not building demodatv (ENABLE_CHANNELRX_DEMODATV=${ENABLE_CHANNELRX_DEMODATV})")
endif()

# need ffmpeg 3.1 that correstonds to
# libavutil 55.27.100
# libavcodec 57.48.101
# libavformat 57.40.101
