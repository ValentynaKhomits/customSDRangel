project(demod)

if (ENABLE_CHANNELRX_HEATMAP)
    add_subdirectory(heatmap)
else()
    message(STATUS "Not building heatmap (ENABLE_CHANNELRX_HEATMAP=${ENABLE_CHANNELRX_HEATMAP})")
endif()

if (ENABLE_CHANNELRX_CHANALYZER)
    add_subdirectory(chanalyzer)
else()
    message(STATUS "Not building chanalyzer (ENABLE_CHANNELRX_CHANALYZER=${ENABLE_CHANNELRX_CHANALYZER})")
endif()

if (ENABLE_CHANNELRX_DEMODATV)
    add_subdirectory(demodatv)
else()
    message(STATUS "Not building demodatv (ENABLE_CHANNELRX_DEMODATV=${ENABLE_CHANNELRX_DEMODATV})")
endif()

# need ffmpeg 3.1 that correstonds to
# libavutil 55.27.100
# libavcodec 57.48.101
# libavformat 57.40.101
find_package(FFmpeg COMPONENTS AVCODEC AVFORMAT AVUTIL SWSCALE)

if (ENABLE_CHANNELRX_DEMODDATV AND FFMPEG_FOUND)
    if(WIN32)
    add_subdirectory(demoddatv)
    else()
    if(((AVUTIL_VERSION VERSION_GREATER "55.27.99") AND (AVCODEC_VERSION VERSION_GREATER "57.48.101")) OR FFMPEG_EXTERNAL OR FFMPEG_SKIP_CHECK)
        message(STATUS "Include demoddatv")
        add_subdirectory(demoddatv)
    else()
        message(STATUS "FFmpeg too old to compile demoddatv; needs at least avutil: 55.27.100, avcodec/avformat: 57.48.101")
    endif()
    endif()
else()
    message(STATUS "Not building demoddatv (ENABLE_CHANNELRX_DEMODDATV=${ENABLE_CHANNELRX_DEMODDATV} FFMPEG_FOUND=${FFMPEG_FOUND})")
endif()
