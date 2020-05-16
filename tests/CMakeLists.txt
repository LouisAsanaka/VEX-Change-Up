cmake_minimum_required(VERSION 3.7)

project(UnitTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(
	"../include"
)

add_library(pathgen STATIC
	../src/libraidzero/planner/profilePlanner.cpp 
	../src/libraidzero/planner/polynomialFunction.cpp
	../src/libraidzero/planner/hermiteInterpolator.cpp
)
target_include_directories(pathgen PUBLIC "../include")

add_executable(UnitTests
	testing.cpp
)

target_link_libraries(UnitTests pathgen ${CMAKE_SOURCE_DIR}/lib/OkapiLibV5.lib)
