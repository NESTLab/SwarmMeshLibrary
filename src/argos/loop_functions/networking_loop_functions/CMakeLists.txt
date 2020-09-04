set(networking_loop_functions_SOURCES networking_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(networking_loop_functions_SOURCES
    ${networking_loop_functions_SOURCES}
    networking_qtuser_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(networking_loop_functions MODULE ${networking_loop_functions_SOURCES})
target_link_libraries(networking_loop_functions
  swarmmesh_controller
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(networking_loop_functions argos3plugin_simulator_qtopengl ${ARGOS_QTOPENGL_LIBRARIES})
endif(ARGOS_COMPILE_QTOPENGL)
