cmake_minimum_required(VERSION 3.7)

project(UnitTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
	"./include"
)

add_library(pathgen STATIC
    ../src/libraidzero/util/miscUtil.cpp
	../src/libraidzero/planner/profilePlanner.cpp 
	../src/libraidzero/planner/polynomialFunction.cpp
	../src/libraidzero/planner/hermiteInterpolator.cpp
    ../src/libraidzero/geometry/rotation2d.cpp
    ../src/libraidzero/geometry/translation2d.cpp
    ../src/libraidzero/geometry/transform2d.cpp
    ../src/libraidzero/geometry/pose2d.cpp
    ../src/libraidzero/trajectory/trajectory.cpp
)
target_include_directories(pathgen PUBLIC "../include")

file(GLOB_RECURSE TEST_SOURCES "./src/*.cpp")

add_executable(UnitTests ${TEST_SOURCES})

#target_link_libraries(UnitTests pathgen ${CMAKE_SOURCE_DIR}/lib/OkapiLibV5.lib)
target_link_libraries(UnitTests pathgen ${CMAKE_SOURCE_DIR}/lib/libOkapiLibV5.a)
