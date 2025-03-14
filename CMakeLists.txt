cmake_minimum_required(VERSION 3.10.0)
project(test_alsa VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# asio
add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${CMAKE_SOURCE_DIR}/asio-1.30.2/include")

if(MSVC)
   target_compile_definitions(asio INTERFACE _WIN32_WINNT=0x0601)
endif(MSVC)

set(WASAPI_LIB ksuser mfplat mfuuid wmcodecdspuuid Winmm)
aux_source_directory(${CMAKE_SOURCE_DIR}/source TRANS_FILES)
add_library(tinyaudio STATIC ${TRANS_FILES})
target_link_libraries(tinyaudio asio)
target_link_libraries(tinyaudio $<$<PLATFORM_ID:Windows>:${WASAPI_LIB}>)
target_link_libraries(tinyaudio $<$<PLATFORM_ID:Linux>:asound>)
target_compile_definitions(tinyaudio PUBLIC $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>)
target_include_directories(tinyaudio PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_include_directories(tinyaudio PUBLIC "${CMAKE_SOURCE_DIR}/source")

add_executable(demo main.cpp)
target_link_libraries(demo tinyaudio)