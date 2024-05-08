cmake_minimum_required(VERSION 2.6)

project(ceres-tests)

list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/third_party/eigen/install")
list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/third_party/ceres-solver/install")

find_package(ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

macro (CERES_TEST NAME)
  add_executable(${NAME}_test tests/${NAME}_test.cpp)
  # Pull in local headers from the generated test directories when ceres_test()
  # is invoked there, as well as the private headers in this directory which
  # may be referenced without the 'ceres' path prefix and all private
  # dependencies that may be directly referenced.
  target_include_directories(${NAME}_test
    PRIVATE ${Ceres_SOURCE_DIR}/internal/ceres
            ${CERES_LIBRARY_PRIVATE_DEPENDENCIES_INCLUDE_DIRS})
  # Some tests include direct references/includes of private dependency
  # headers which are not propagated via the ceres targets, so link them
  # explicitly.
  target_link_libraries(${NAME}_test PRIVATE Ceres::ceres)

  # covariance_test uses SuiteSparseQR.hpp. However, since SuiteSparse import
  # targets are private (link only) dependencies not propagated to consumers,
  # we need to link against the target explicitly here.
  add_test(NAME ${NAME}_test
            COMMAND ${NAME}_test)
  add_custom_command(
    TARGET ${NAME}_test
    COMMENT "Run tests"
    POST_BUILD 
    COMMAND ${NAME}_test
  )
endmacro (CERES_TEST)

CERES_TEST(mocap_calibration)
CERES_TEST(mocap_wand_calibration)
