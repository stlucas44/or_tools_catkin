cmake_minimum_required(VERSION 3.16.3)
project(or_tools_catkin)

find_package(catkin REQUIRED COMPONENTS
            rosconsole
            roslib
            )

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

set(ortools_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/libs/or_tools/include)
set(ortools_LIBRARIES ${CMAKE_SOURCE_DIR}/libs/or_tools/lib/libortools.so)

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${ortools_INCLUDE_DIRS} # does it need all or is include sufficient)
        LIBRARIES ${PROJECT_NAME}
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${ortools_INCLUDE_DIRS}
                    )

add_library(${PROJECT_NAME}
  src/or_interface.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${ortools_LIBRARIES})

catkin_add_gtest(test-orlib test/orlib-test.cpp)
target_link_libraries(test-orlib ${catkin_LIBRARIES} ${ortools_LIBRARIES})

catkin_add_gtest(test_orinterface test/orinterface-test.cpp)
target_link_libraries(test_orinterface ${PROJECT_NAME})

# find_package -> looks for FindSomeThing.cmake, implicitly calls find_path and find library
