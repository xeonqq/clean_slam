find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED
        slam_core.cpp
        orb_extractor.cpp
        slam_system.cpp dataset_loader.cpp
        ground_truth.cpp cv_utils.cpp orb_feature_matcher.cpp transfer_error.cpp transfer_error.h camera_motion_estimator.cpp camera_motion_estimator.h homogeneous_matrix.h homography_motion_estimator.cpp epipolar_constraint_motion_estimator.cpp homogeneous_matrix.cpp stamped_transformation.h eigen_utils.h eigen_utils.cpp epipolar_constraint_motion_estimator.h homography_motion_estimator.h i_projective_transformation.cpp i_projective_transformation.h cv_algorithms.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/third_party/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/third_party/g2o/lib/libg2o.so
        )
