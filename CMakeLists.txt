add_library(pico_sgp40 INTERFACE)

target_sources(pico_sgp40 INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/src/SGP40.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/sensirion_voc_algorithm.c
)

target_include_directories(pico_sgp40 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src ${CMAKE_CURRENT_LIST_DIR}/include)

# Pull in pico libraries that we need
target_link_libraries(pico_sgp40 INTERFACE pico_stdlib hardware_i2c)


