cmake_minimum_required(VERSION 3.3)

#set(CMAKE_INSTALL_PREFIX ~/swarmmeshbundle)

#project(SwarmMeshLibrary)

message("${CMAKE_INSTALL_PREFIX}")

#
# Compile libswarmmesh
#

add_library(swarm_mesh_library SHARED
swarmmesh.h swarmmesh.cpp
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

install(TARGETS swarm_mesh_library EXPORT SwarmMeshTargets
  LIBRARY DESTINATION lib/SwarmMeshLibrary
  ARCHIVE DESTINATION lib/SwarmMeshLibrary
  RUNTIME DESTINATION bin/
)
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/../swarmmesh.h" DESTINATION include/SwarmMeshLibrary
)


# function(my_export_target _target _include_dir)
#     file(
#         WRITE "${CMAKE_CURRENT_BINARY_DIR}/${_target}Config.cmake"
#         "
#             include(\"\$\{CMAKE_CURRENT_LIST_DIR\}/${_target}Targets.cmake\")
#             set_property(
#                 TARGET ${_target}
#                 APPEND PROPERTY
#                     INTERFACE_INCLUDE_DIRECTORIES \"${_include_dir}\"
#             )
#         "
#     )
#     export(TARGETS ${_target} FILE "${CMAKE_CURRENT_BINARY_DIR}/${_target}Targets.cmake")
#     export(PACKAGE ${_target})
# endfunction(my_export_target)


#  message("${CMAKE_CURRENT_SOURCE_DIR}")

# my_export_target(swarm_mesh_library "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")

# message("Includes")
# message("${swarm_mesh_library_INCLUDE_DIR}")
