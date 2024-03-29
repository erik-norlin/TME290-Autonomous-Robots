# Copyright (C) 2018 Ola Benderius
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.2)

project(opendlv-virtual-space)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set-v0.9.10.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.145.hpp)

################################################################################
# This project requires C++14 or newer.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
message("-- Found ccache, will be using that.") 
set(CMAKE_CXX_COMPILER_LAUNCHER "ccache")
endif(CCACHE_FOUND)
# Strip unneeded symbols from binaries.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
# Build a static binary.
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
# Add further warning levels.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -D_XOPEN_SOURCE=700 \
    -D_FORTIFY_SOURCE=2 \
    -O2 \
    -fstack-protector \
    -fomit-frame-pointer \
    -pipe \
    -pedantic -pedantic-errors \
    -Werror \
    -Weffc++ \
    -Wall -Wextra -Wshadow -Wdeprecated \
    -Wdiv-by-zero -Wfloat-equal -Wfloat-conversion -Wsign-compare -Wpointer-arith \
    -Wuninitialized -Wunreachable-code \
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-but-set-parameter -Wunused-but-set-variable \
    -Wunused-value -Wunused-variable -Wunused-result \
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")
# Threads are necessary for linking the resulting binaries as UDPReceiver is running in parallel.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Avoid compiling cluon-msc if it is installed already
find_program(CLUON_PATH cluon-msc)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE}
)
if(CLUON_PATH)
  message("-- Found cluon installed at " ${CLUON_PATH}) 
  add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CLUON_PATH} ${CMAKE_BINARY_DIR}/cluon-msc
    DEPENDS ${CMAKE_BINARY_DIR}/cluon-complete.hpp
  )
else()
  message("-- Generating fresh cluon-msc") 
  add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/src/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.cpp 
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc   ${CMAKE_BINARY_DIR}/cluon-complete.cpp -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_BINARY_DIR}/cluon-complete.hpp
  )
endif()


################################################################################
# Generate opendlv-standard-message-set.hpp from ${OPENDLV_STANDARD_MESSAGE_SET} file.
# From libcluon v0.0.106 on, we don't generate opendlv-standard-message-set.cpp any longer
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
  COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp  ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/${OPENDLV_STANDARD_MESSAGE_SET} 
  ${CMAKE_BINARY_DIR}/cluon-msc)
# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)


find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
set(LIBRARIES ${LIBRARIES} ${EIGEN3_LIBRARIES})

################################################################################
# Gather all object code first to avoid double compilation.
add_library(${PROJECT_NAME}-core OBJECT 
  ${CMAKE_BINARY_DIR}/cluon-complete.hpp
  ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/world-object.cpp 
)
set(LIBRARIES Threads::Threads)

################################################################################
# Create executable.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

################################################################################
# Enable unit testing.
enable_testing()
add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/test-world-object.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
target_link_libraries(${PROJECT_NAME}-runner ${LIBRARIES})
add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)

################################################################################
# Install executable.
install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})
