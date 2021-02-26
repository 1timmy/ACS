cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 14)

project (ACS)

add_executable(${PROJECT_NAME} src/Main.cpp
                               src/VisionAlgorithm.cpp)

include_directories(include)
include_directories("/usr/include/spinnaker")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

find_library(SPIN_LIB Spinnaker REQUIRED)

pkg_check_modules(libmodbus REQUIRED IMPORTED_TARGET libmodbus)

target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE PkgConfig::libmodbus)
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(${PROJECT_NAME} LINK_PRIVATE ${SPIN_LIB})