include_directories("${CMAKE_SOURCE_DIR}/source")

add_executable(test_encoder test_encoder.cpp)
target_link_libraries(test_encoder bettyaudio)

add_executable(test_network test_network.cpp)
target_link_libraries(test_network bettyaudio)

add_executable(test_player test_player.cpp)
target_link_libraries(test_player bettyaudio)

add_executable(test_audio_process test_audio_process.cpp)
target_link_libraries(test_audio_process bettyaudio)