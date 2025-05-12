include_directories("${CMAKE_SOURCE_DIR}/source")

add_executable(test_encoder test_encoder.cpp)
target_link_libraries(test_encoder bettyaudio)

add_executable(test_network test_network.cpp)
target_link_libraries(test_network bettyaudio)

add_executable(test_player test_player.cpp)
target_link_libraries(test_player bettyaudio)

# add_executable(test_audio_process test_audio_process.cpp)
# target_link_libraries(test_audio_process bettyaudio)

# add_executable(test_dr_compressor test_dr_compressor.cpp)
# target_link_libraries(test_dr_compressor bettyaudio)

# add_executable(test_locsample test_locsampler.cpp)
# target_link_libraries(test_locsample bettyaudio)

# add_executable(test_filter_banks test_filter_banks.cpp)
# target_link_libraries(test_filter_banks bettyaudio)