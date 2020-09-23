include(ExternalProject)
set(DATASET_DIR ${CMAKE_BINARY_DIR}/test/dataset)
set(CONFIG_DIR ${CMAKE_SOURCE_DIR}/config)
set(TEST_DIR ${CMAKE_SOURCE_DIR}/test)
ExternalProject_Add(
        rgbd_dataset_freiburg1_xyz
        PREFIX ""
        URL https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
        URL_HASH SHA512=e77dcdabf097f835973edfef920246788dd26da1315d74f3b36799e8f1ab43da9b2326eb32613fa696cd8efab7ce83f0472aae21a46a62b3ab67859c7d4692a9
        DOWNLOAD_DIR ${DATASET_DIR}
        SOURCE_DIR ${DATASET_DIR}/rgbd_dataset_freiburg1_xyz
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
)
ExternalProject_Add(
        freiburg3_long_office_household
        PREFIX ""
        URL https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.tgz
        URL_HASH SHA512=630e3a2d262ae4220d6323c17e8e48e2e91c906dcbe0fe8fe60b0099272a8c626338f4d20f379adcdadd125361004f14ef8efa49b7c877413de36bd86dbe82da
        DOWNLOAD_DIR ${DATASET_DIR}
        SOURCE_DIR ${DATASET_DIR}/freiburg3_long_office_household
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
)


add_compile_definitions(DATASET_DIR="${DATASET_DIR}")
add_compile_definitions(CONFIG_DIR="${CONFIG_DIR}")
add_compile_definitions(TEST_DIR="${TEST_DIR}")
include_directories(
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/include
)

set(BINARY ${CMAKE_PROJECT_NAME}_test)
add_executable(${BINARY} test_dataset_loader.cpp test_slam_system_freiburg.cpp test_slam_system_homography.cpp test_opencv_functions.cpp test_search_by_projection.cpp test_map_point.cpp test_frame.cpp test_cv_utils..cpp test_local_mapping.cpp)
add_test(NAME ${BINARY} COMMAND ${BINARY})
target_link_libraries(
        ${BINARY}
        PUBLIC ${CMAKE_PROJECT_NAME}
        gtest_main
        gmock
)
add_dependencies(${BINARY} rgbd_dataset_freiburg1_xyz freiburg3_long_office_household)
