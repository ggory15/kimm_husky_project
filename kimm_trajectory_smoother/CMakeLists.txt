cmake_minimum_required(VERSION 3.0.2)

SET(PROJECT_NAME kimm_trajectory_smoother)
SET(PROJECT_DESCRIPTION "kimm_trajectory with time opt ")
SET(PROJECT_URL "http://github.com/ggory15/kimm_trajectory")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)
add_compile_options(-std=c++17 -O3)

#INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

find_package(catkin REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 DEPENDS system_lib
)

include_directories(
  include
)

# --- OPTIONS ----------------------------------------
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
OPTION (BUILD_UNIT_TESTS "Build the unitary tests" ON)
OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION (EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)
OPTION (EIGEN_NO_AUTOMATIC_RESIZING "If ON, it forbids automatic resizing of dynamics arrays and matrices" OFF)

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

IF(EIGEN_RUNTIME_NO_MALLOC)
  MESSAGE(STATUS "Option EIGEN_RUNTIME_NO_MALLOC on.")
  ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
ENDIF(EIGEN_RUNTIME_NO_MALLOC)

IF(EIGEN_NO_AUTOMATIC_RESIZING)
  MESSAGE(STATUS "Option EIGEN_NO_AUTOMATIC_RESIZING on.")
  ADD_DEFINITIONS(-DEIGEN_NO_AUTOMATIC_RESIZING)
ENDIF(EIGEN_NO_AUTOMATIC_RESIZING)

find_package(Eigen3 REQUIRED PKG_CONFIG_REQUIRES "eigen3 >= 3.2.0")
find_package(Boost REQUIRED COMPONENTS system)

FINDPYTHON(2.7 REQUIRED)
SET(PYWRAP ${PROJECT_NAME}_pywrap)
find_package(eigenpy REQUIRED)
SEARCH_FOR_BOOST_PYTHON(REQUIRED)

SET(${PROJECT_NAME}_MATH_HEADERS
  include/kimm_trajectory_smoother/Path.h
  include/kimm_trajectory_smoother/Trajectory.h
  include/kimm_trajectory_smoother/bindings/expose_Path.hpp
  include/kimm_trajectory_smoother/bindings/expose_Trajectory.hpp
  include/kimm_trajectory_smoother/bindings/fwd.hpp
  include/kimm_trajectory_smoother/bindings/Path.hpp
  include/kimm_trajectory_smoother/bindings/Trajectory.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
)

SET(${PROJECT_NAME}_SOURCES
  src/Path.cpp
  src/Trajectory.cpp
  )

ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)


ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC)

#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
#INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)


#INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})

