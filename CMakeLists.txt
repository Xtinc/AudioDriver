cmake_minimum_required(VERSION 3.10.0)
project(test_alsa VERSION 0.1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_library(ASOUND_LIB asound)
    find_library(UDEV_LIB udev)
    if(NOT ASOUND_LIB)
        message(FATAL_ERROR "Could not find asound library. Please install the necessary development packages.")
        return()
    endif()
        if(NOT UDEV_LIB)
        message(FATAL_ERROR "Could not find udev library. Please install the necessary development packages.")
        return()
    endif()
endif()

# Compiler-specific options
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    # Set warning level to W4 and enable address sanitizer for MSVC
    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    add_compile_options(/fsanitize=address)
    message(STATUS "MSVC compiler: Address sanitizer enabled")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Enable all warnings for GCC/Clang
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Platform-specific libraries
set(WINDOWS_LIBS ksuser mfplat mfuuid wmcodecdspuuid Winmm)
set(LINUX_LIBS asound udev pthread)

# ASIO library setup
add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${CMAKE_SOURCE_DIR}/asio-1.30.2/include")
target_compile_definitions(asio INTERFACE $<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601>)

#opus
add_subdirectory(opus-1.5.2)

# Collect source files
aux_source_directory(${CMAKE_SOURCE_DIR}/source TRANS_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/source/ns ANS_FILES)

# Main library
add_library(bettyaudio STATIC ${TRANS_FILES} ${ANS_FILES})
target_include_directories(bettyaudio 
    PRIVATE "${CMAKE_SOURCE_DIR}/source"
    PRIVATE "${CMAKE_SOURCE_DIR}/source/ns"
    PUBLIC "${CMAKE_SOURCE_DIR}/include"
)
target_link_libraries(bettyaudio 
    asio
    opus
    $<$<PLATFORM_ID:Windows>:${WINDOWS_LIBS}>
    $<$<PLATFORM_ID:Linux>:${LINUX_LIBS}>
)
target_compile_definitions(bettyaudio 
    PUBLIC $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>
)

# Tests
add_subdirectory(test)