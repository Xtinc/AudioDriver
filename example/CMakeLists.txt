cmake_minimum_required(VERSION 3.10.0)
project(AudioExample VERSION 1.0.0 LANGUAGES CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set bettyaudio library path (adjust this path to your actual library location)
set(BETTYAUDIO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "Path to bettyaudio")

# Find bettyaudio library
find_library(BETTYAUDIO_LIB
    NAMES bettyaudio libbettyaudio
    PATHS "${BETTYAUDIO_DIR}/build/Debug" "${BETTYAUDIO_DIR}/build/Release"
    PATH_SUFFIXES Debug Release
    REQUIRED
)

# Platform-specific libraries
if(WIN32)
    set(PLATFORM_LIBS ksuser mfplat mfuuid wmcodecdspuuid Winmm)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ALSA REQUIRED alsa)
    pkg_check_modules(UDEV REQUIRED libudev)
    set(PLATFORM_LIBS pthread ${ALSA_LIBRARIES} ${UDEV_LIBRARIES})
endif()

# Create executable
add_executable(audio_example main.cpp)

target_include_directories(audio_example PRIVATE 
    "${BETTYAUDIO_DIR}/include"
)

target_link_libraries(audio_example 
    ${BETTYAUDIO_LIB}
    ${PLATFORM_LIBS}
)

target_compile_definitions(audio_example PRIVATE
    $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>
)

message(STATUS "AudioExample: Using bettyaudio library: ${BETTYAUDIO_LIB}")
