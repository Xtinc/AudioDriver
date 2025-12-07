include_directories("${CMAKE_SOURCE_DIR}/source")

add_executable(test_network test_network.cpp)
target_link_libraries(test_network bettyaudio)

add_executable(test_player test_player.cpp)
target_link_libraries(test_player bettyaudio)

add_executable(test_audio_process test_audio_process.cpp)
target_link_libraries(test_audio_process bettyaudio)

add_executable(test_dr_compressor test_dr_compressor.cpp)
target_link_libraries(test_dr_compressor bettyaudio)

add_executable(test_locsampler test_locsampler.cpp)
target_link_libraries(test_locsampler bettyaudio)

add_executable(test_lms_filter test_lms_filter.cpp)
target_link_libraries(test_lms_filter bettyaudio)

add_executable(bluetoothctl bluetooth_main.cpp)
target_link_libraries(bluetoothctl bettyaudio)

add_executable(test_rpc test_rpc.cpp)
target_link_libraries(test_rpc bettyaudio)