cmake_minimum_required(VERSION 3.18)
project(ransac-fit-plane)
find_package( Eigen3 3.3 REQUIRED NO_MODULE)
MESSAGE(STATUS "Include dirs ${Eigen_INCLUDE_DIRS}")
add_executable(${PROJECT_NAME} ${PROJECT_NAME}/main.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen )
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17 )
