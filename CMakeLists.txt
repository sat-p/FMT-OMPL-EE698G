#Cmake setup
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_VERBOSE_MAKEFILE ON)

project(ee698g)

find_package(ompl)

#Include the header directory
set(solverdir .)
include_directories(${solverdir}
                    ${ompl_INCLUDE_DIRS})

#Adding sources
set(SOURCES ${solverdir}/code/app/src/Application.cxx
            ${solverdir}/code/planners/src/FMT.cxx
            ${solverdir}/code/app/src/ompl_test.cxx)

#Compile
add_executable(ee698g ${SOURCES})

set_target_properties(ee698g PROPERTIES COMPILE_FLAGS "-Wall -O3 --std=c++11")
target_link_libraries(ee698g ${OMPL_LIBRARIES}) 
