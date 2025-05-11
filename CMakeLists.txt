cmake_minimum_required(VERSION 3.10.0)
project(test_alsa VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译器警告级别为最高
if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  add_compile_options(/fsanitize=address)
  message(STATUS "Address sanitizer enabled (MSVC)")
endif()

# asio
add_library(asio INTERFACE)
target_include_directories(asio INTERFACE "${CMAKE_SOURCE_DIR}/asio-1.30.2/include")
target_compile_definitions(asio INTERFACE  $<$<CXX_COMPILER_ID:MSVC>:_WIN32_WINNT=0x0601>)

set(WINDWOS_LIB ksuser mfplat mfuuid wmcodecdspuuid Winmm)
set(LINUX_LIB asound udev pthread)
aux_source_directory(${CMAKE_SOURCE_DIR}/source TRANS_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/source/ns ANS_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/source/utils UTIL_FILES)
add_library(bettyaudio STATIC ${TRANS_FILES} ${ANS_FILES} ${UTIL_FILES})
target_include_directories(bettyaudio PRIVATE "${CMAKE_SOURCE_DIR}/source")
target_link_libraries(bettyaudio asio)
target_link_libraries(bettyaudio $<$<PLATFORM_ID:Windows>:${WINDWOS_LIB}>)
target_link_libraries(bettyaudio $<$<PLATFORM_ID:Linux>:${LINUX_LIB}>)
target_compile_definitions(bettyaudio PUBLIC $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>)
target_compile_definitions(bettyaudio PRIVATE ENABLE_DBG_ASSERT)
target_include_directories(bettyaudio PUBLIC "${CMAKE_SOURCE_DIR}/include")

add_subdirectory(test)