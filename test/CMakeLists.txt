add_executable(test_encoder test_encoder.cpp)
target_include_directories(test_encoder PRIVATE "${CMAKE_SOURCE_DIR}/source")
target_link_libraries(test_encoder bettyaudio)