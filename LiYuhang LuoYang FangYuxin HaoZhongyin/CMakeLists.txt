cmake_minimum_required(VERSION 3.1)
SET(CMAKE_BUILD_TYPE "Debug") # "Debug" "Release"
PROJECT(Serial)

find_package(OpenCV 4.5.0)

message(STATUS "OpenCV library status:")
message(STATUS "	config: ${OpenCV_DIR}")
message(STATUS "	version: ${OpenCV_VERSION}")
message(STATUS "	libraries: ${OpenCV_LIBS}")
message(STATUS "	include path: ${OpenCV_INCLUDE_DIRS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native")
ADD_EXECUTABLE(Serial 
                    ./main.cpp
                    ./serialport.cpp
                    ./serialport.h
                    ./CRC_Check.cpp
                    ./CRC_Check.h
                   
                    )

#link your application with OpencvCV libraries
target_link_libraries(Serial PRIVATE ${OpenCV_LIBS})
