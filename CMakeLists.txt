cmake_minimum_required(VERSION 3.10.0)
project(test_alsa VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# asio
add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${CMAKE_SOURCE_DIR}/asio-1.30.2/include")
target_compile_definitions(asio INTERFACE  $<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601>)

set(WINDWOS_LIB ksuser mfplat mfuuid wmcodecdspuuid Winmm)
set(LINUX_LIB asound udev pthread)
aux_source_directory(${CMAKE_SOURCE_DIR}/source TRANS_FILES)
add_library(bettyaudio STATIC ${TRANS_FILES})
target_link_libraries(bettyaudio asio)
target_link_libraries(bettyaudio $<$<PLATFORM_ID:Windows>:${WINDWOS_LIB}>)
target_link_libraries(bettyaudio $<$<PLATFORM_ID:Linux>:${LINUX_LIB}>)
target_compile_definitions(bettyaudio PUBLIC $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>)
target_include_directories(bettyaudio PUBLIC "${CMAKE_SOURCE_DIR}/include")
# target_include_directories(bettyaudio PUBLIC "${CMAKE_SOURCE_DIR}/source")

add_subdirectory(test)