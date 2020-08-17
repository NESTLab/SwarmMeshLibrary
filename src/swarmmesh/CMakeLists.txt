# cmake_minimum_required(VERSION 3.3)

# project(SwarmMeshLibrary)

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

#
# Compile libswarmmesh
#
add_library(swarm_mesh_library SHARED
swarmmesh.h swarmmesh.cpp
)
set_target_properties(swarm_mesh_library PROPERTIES CXX_STANDARD 17)
target_compile_features(swarm_mesh_library PUBLIC cxx_std_17)

# message("${CMAKE_CURRENT_SOURCE_DIR}")

# my_export_target(swarm_mesh_library "${CMAKE_CURRENT_SOURCE_DIR}")
