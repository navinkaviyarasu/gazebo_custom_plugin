cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(TVCPlugin)

# --- 1. Find Gazebo Sim (Harmonic/System) Packages ---
# Using the highest detected version numbers from your system's /usr/share/gz directory
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED) # Also updated to plugin2

# --- 2. Define the Plugin Library ---
add_library(TVCPlugin SHARED TVCPlugin.cc)

# --- 3. Link Libraries ---
target_link_libraries(TVCPlugin PUBLIC
  # Core Gazebo Sim component (Version 8)
  gz-sim8::gz-sim8
  # Gazebo Transport for messaging (Version 13)
  gz-transport13::gz-transport13
  # Gazebo Messages for the data types (Version 10)
  gz-msgs10::gz-msgs10
  # Gazebo Math for Vector3d, Pose3, etc. (Version 7)
  gz-math7::gz-math7
  # Gazebo Utils for Plugin Registration (Version 2)
  gz-plugin2::gz-plugin2
)

# --- 4. Install the Plugin ---
# We will use the generic 'lib' directory now, and rely on the environment 
# variable GZ_SIM_SYSTEM_PLUGIN_PATH being set correctly.
install(TARGETS TVCPlugin
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/gz-sim-8
)
