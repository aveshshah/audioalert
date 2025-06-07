cmake_minimum_required(VERSION 3.8)
project(audio_alert_cpp)

# Enable C++17 for <filesystem>
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(audioalert_msg REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# pkg-config to locate ALSA and libsndfile
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA    REQUIRED alsa)
pkg_check_modules(SNDFILE REQUIRED sndfile)

add_executable(fleet_playback src/fleet_playback.cpp)

ament_target_dependencies(fleet_playback
  rclcpp
  audioalert_msg
  ament_index_cpp
  nlohmann_json
)

# Include dirs for ALSA & libsndfile
target_include_directories(fleet_playback
  PRIVATE
    ${ALSA_INCLUDE_DIRS}
    ${SNDFILE_INCLUDE_DIRS}
)

# link libraries (plain signature to match ament_target_dependencies)
target_link_libraries(fleet_playback
   ${ALSA_LIBRARIES}
   ${SNDFILE_LIBRARIES}
)

install(TARGETS fleet_playback
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

