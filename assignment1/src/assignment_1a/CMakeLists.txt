cmake_minimum_required(VERSION 3.2)
project(helloworld)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

find_package(Threads REQUIRED)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/messages.hpp
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/messages.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/messages.odvd)
include_directories(SYSTEM ${CMAKE_BINARY_DIR})

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/helloworld.cpp 
${CMAKE_CURRENT_SOURCE_DIR}/prime-checker.cpp ${CMAKE_BINARY_DIR}/messages.hpp)
target_link_libraries(${PROJECT_NAME} Threads::Threads)

enable_testing()
add_executable(test-prime-checker test-prime-checker.cpp ${CMAKE_CURRENT_SOURCE_DIR}/prime-checker.cpp)
add_test(NAME test-prime-checker COMMAND test-prime-checker)
