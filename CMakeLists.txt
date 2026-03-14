cmake_minimum_required(VERSION 3.20)
project(wsv_robot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(AWSCR_BUILD_PYTHON "Build Python bindings via pybind11" ON)

add_library(awscr_core
  core/src/dynamics_3dof.cpp
  core/src/estimator_es_ekf.cpp
  core/src/controller_smc.cpp
  core/src/brain.cpp
  core/src/planner_bcd.cpp
  core/src/planner_intercept_dubins.cpp
  core/src/vfh_plus.cpp
)

target_include_directories(awscr_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/include
)

if(AWSCR_BUILD_PYTHON)
  set(PYBIND11_FINDPYTHON ON)
  set(Python3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.venv/include")
  find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG QUIET)
  if(pybind11_FOUND)
    pybind11_add_module(pyawscr bindings/pyawscr.cpp)
    target_link_libraries(pyawscr PRIVATE awscr_core)
  endif()
endif()

