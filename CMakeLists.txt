cmake_minimum_required(VERSION 3.10)
project(accdecel)

set(CMAKE_CXX_STANDARD 14)

#find_package( QtWidgets REQUIRED )

include_directories(.)

add_executable(accdecel main-stdout.cpp)

#target_link_libraries( accel_decel Qt5::Widgets )

