cmake_minimum_required(VERSION 3.10.0)
project(test_alsa VERSION 0.1.0 LANGUAGES C CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required libraries
set(AUDIO_DRIVER_DEP_LIBS TRUE)
if(WIN32)
    set(PLATFORM_LIBS ksuser mfplat mfuuid wmcodecdspuuid Winmm)
else()
    find_package(PkgConfig QUIET)
    pkg_check_modules(ALSA QUIET alsa)
    pkg_check_modules(UDEV QUIET libudev)
    pkg_check_modules(DBUS QUIET dbus-1)

    set(PLATFORM_LIBS pthread)

    if(ALSA_FOUND AND UDEV_FOUND)
        list(APPEND PLATFORM_LIBS ${ALSA_LIBRARIES} ${UDEV_LIBRARIES})
    else()
        set(AUDIO_DRIVER_DEP_LIBS FALSE)
        message(STATUS "AudioDriver: ALSA or UDEV not found, skipping Linux build")
    endif()

    if(DBUS_FOUND)
        list(APPEND PLATFORM_LIBS ${DBUS_LIBRARIES})
    else()
        message(STATUS "AudioDriver: DBUS not found, building without DBUS support")
    endif()
endif()

# Only proceed with build if libraries are available
if(AUDIO_DRIVER_DEP_LIBS)
    # Compiler options
    set(GNU_COMPILER_OPTIONS "-Wall -Wextra -Wpedantic -Wshadow")
    add_compile_options($<$<CXX_COMPILER_ID:MSVC>:/W4>)
    add_compile_options($<$<CXX_COMPILER_ID:GNU,Clang>:${GNU_COMPILER_OPTIONS}>)

    # asio
    add_library(asio INTERFACE)
    target_include_directories(asio INTERFACE asio-1.30.2/include)
    target_compile_definitions(asio INTERFACE $<$<PLATFORM_ID:Windows>:_WIN32_WINNT=0x0601>)

    # opus
    add_subdirectory(opus-1.5.2)

    aux_source_directory(source SOURCES)
    aux_source_directory(source/bt BT_SOURCES)

    # Main library
    add_library(bettyaudio STATIC ${SOURCES} ${BT_SOURCES})
    target_include_directories(bettyaudio 
        PRIVATE source
        PUBLIC include
        PUBLIC $<$<NOT:$<PLATFORM_ID:Windows>>:${ALSA_INCLUDE_DIRS}>
        PUBLIC $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<BOOL:${DBUS_FOUND}>>:${DBUS_INCLUDE_DIRS}>
    )
    target_link_libraries(bettyaudio asio opus ${PLATFORM_LIBS})
    target_compile_definitions(bettyaudio 
        PUBLIC $<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>
        PUBLIC $<$<AND:$<NOT:$<PLATFORM_ID:Windows>>,$<BOOL:${DBUS_FOUND}>>:HAVE_DBUS>
    )

    # Tests
    add_subdirectory(test)
    
    message(STATUS "AudioDriver: Build targets created successfully")
else()
    message(STATUS "AudioDriver: Skipping all targets due to missing dependencies")
endif()