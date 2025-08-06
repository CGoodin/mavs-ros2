#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mavs-ros2::mavs-ros2" for configuration ""
set_property(TARGET mavs-ros2::mavs-ros2 APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(mavs-ros2::mavs-ros2 PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libmavs-ros2.a"
  )

list(APPEND _cmake_import_check_targets mavs-ros2::mavs-ros2 )
list(APPEND _cmake_import_check_files_for_mavs-ros2::mavs-ros2 "${_IMPORT_PREFIX}/lib/libmavs-ros2.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
