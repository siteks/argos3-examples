add_library(xpuck_remote MODULE xpuck_remote.h xpuck_remote.cpp)
target_link_libraries(xpuck_remote
  zmq
  argos3core_simulator
  argos3plugin_simulator_epuck
  argos3plugin_simulator_genericrobot)
