cmake_minimum_required(VERSION 3.5)
project(libinrol_sim_peginhole)

add_library(inrol_sim_peginhole STATIC IMPORTED)
set_property(
  TARGET inrol_sim_peginhole PROPERTY
  IMPORTED_LOCATION ${CMAKE_CURRENT_LIST_DIR}/lib/libinrol_sim_peginhole.a)
set_property(
  TARGET inrol_sim_peginhole APPEND PROPERTY
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/include)