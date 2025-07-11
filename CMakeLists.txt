CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(urdfparser)

FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)

SET(CMAKE_CXX_STANDARD 17)

SET(URDF_SRCS
  src/common.cpp
  src/joint.cpp
  src/geometry.cpp
  src/link.cpp
  src/model.cpp
  src/tinyxml.cpp
  src/tinyxmlerror.cpp
  src/tinyxmlparser.cpp
)
ADD_LIBRARY( urdfparser ${URDF_SRCS})

target_include_directories(urdfparser PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(urdfparser PUBLIC
    Eigen3::Eigen
    Boost::boost
)

SET(URDFPARSER_LIB urdfparser)

SET_PROPERTY(TARGET urdfparser PROPERTY POSITION_INDEPENDENT_CODE ON)

OPTION(URDF_BUILD_TEST "enable testing of library" OFF)

IF(URDF_BUILD_TEST) 
  include_directories(".")  # include catch.hpp
  
  ADD_EXECUTABLE(parse_simple test/parse_simple.cpp)
  TARGET_LINK_LIBRARIES(parse_simple urdfparser)

  ADD_EXECUTABLE(load_file test/load_file.cpp)
  TARGET_LINK_LIBRARIES(load_file urdfparser)
  
  add_executable(urdf_test main_test.cpp)
  TARGET_LINK_LIBRARIES(urdf_test urdfparser)
ENDIF(URDF_BUILD_TEST)
