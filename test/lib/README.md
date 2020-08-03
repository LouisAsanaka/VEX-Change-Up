To compile OkapiLib for desktop:

- Edit flags to remove gtest and coverage:
    - `set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17 -Wall -Wextra -Wshadow -Wnull-dereference -Wno-psabi -Wno-unused-function -pthread -g -D THREADS_STD")`
- Replace `add_executable(OkapiLibV5` with `add_library(OkapiLibV5 STATIC`
- Remove all test/ files in list except implMocks.cpp
- In implMocks.cpp / implMocks.hpp remove all EXPECT_ to remove gtest
- Remove target_link_libraries

Build with: `cmake -GNinja .` then `ninja`. Copy the resulting libOkapiLibV5.a here.