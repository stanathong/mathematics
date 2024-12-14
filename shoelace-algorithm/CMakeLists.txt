cmake_minimum_required(VERSION 3.18)
project(shoelace-algorithm)
add_executable(${PROJECT_NAME} ${PROJECT_NAME}/main.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17 )
