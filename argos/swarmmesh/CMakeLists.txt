link_directories(${CMAKE_BINARY_DIR}/controllers/swarmmesh_controller)

#
# Compile libswarmmesh
#
add_library(swarmmesh SHARED 
swarmmesh.h swarmmesh.cpp
)#networking.h networking.cpp)

# target_link_libraries(swarmmesh
# swarmmesh_controller)

