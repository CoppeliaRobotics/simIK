cmake_minimum_required(VERSION 3.15)
project(simExtIK)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MACOSX_RPATH 1)

# XXX: without -O2, IK plugin is broken (at least on macOS):
set(CMAKE_CXX_FLAGS "-O2")

find_package(Boost REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        set(LIBPLUGIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../libPlugin)
    endif()
endif()
list(APPEND CMAKE_MODULE_PATH ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)

coppeliasim_add_plugin(
    simExtIK
    LEGACY
    SOURCES
    simExtIK.cpp
    envCont.cpp
    ../coppeliaKinematicsRoutines/ik.cpp
    ../coppeliaKinematicsRoutines/environment.cpp
    ../coppeliaKinematicsRoutines/serialization.cpp
    ../coppeliaKinematicsRoutines/ikGroupContainer.cpp
    ../coppeliaKinematicsRoutines/ikGroup.cpp
    ../coppeliaKinematicsRoutines/ikElement.cpp
    ../coppeliaKinematicsRoutines/objectContainer.cpp
    ../coppeliaKinematicsRoutines/sceneObject.cpp
    ../coppeliaKinematicsRoutines/dummy.cpp
    ../coppeliaKinematicsRoutines/joint.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/MyMath.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/3Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/4Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/7Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/Vector.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/3X3Matrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/4X4Matrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/4X4FullMatrix.cpp
    ${COPPELIASIM_ROOT_DIR}/programming/simMath/MMatrix.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionData.cpp
    ${COPPELIASIM_COMMON_DIR}/scriptFunctionDataItem.cpp
)
target_compile_definitions(simExtIK PRIVATE SIM_MATH_DOUBLE)
target_include_directories(simExtIK PRIVATE ../coppeliaKinematicsRoutines)
target_include_directories(simExtIK PRIVATE ${COPPELIASIM_ROOT_DIR}/programming/simMath)
coppeliasim_add_lua(simIK.lua)
coppeliasim_add_helpfile(simIK.htm)
coppeliasim_add_helpfile(simIK.json SUBDIR index)
