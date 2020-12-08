cmake_minimum_required(VERSION 3.7)

project(UnitTests)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/out)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
	${CMAKE_SOURCE_DIR}/include
)

add_library(pathgen STATIC
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/util/mathUtil.cpp
	${CMAKE_SOURCE_DIR}/../src/libraidzero/planner/profilePlanner.cpp 
	${CMAKE_SOURCE_DIR}/../src/libraidzero/planner/polynomialFunction.cpp
	${CMAKE_SOURCE_DIR}/../src/libraidzero/planner/hermiteInterpolator.cpp
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/geometry/rotation2d.cpp
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/geometry/translation2d.cpp
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/geometry/transform2d.cpp
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/geometry/pose2d.cpp
    ${CMAKE_SOURCE_DIR}/../src/libraidzero/trajectory/trajectory.cpp
)
target_include_directories(pathgen PUBLIC ${CMAKE_SOURCE_DIR}/../include)

file(GLOB_RECURSE TEST_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

add_executable(UnitTests ${TEST_SOURCES})

target_link_libraries(UnitTests pathgen ${CMAKE_SOURCE_DIR}/lib/libOkapiLibV5.a)
