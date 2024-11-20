# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "src/CMakeFiles/obs_person_tracker_autogen.dir/AutogenUsed.txt"
  "src/CMakeFiles/obs_person_tracker_autogen.dir/ParseCache.txt"
  "src/obs_person_tracker_autogen"
  )
endif()
