find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost 1.72 COMPONENTS program_options REQUIRED)

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/third_party
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/include
        ${Boost_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} SHARED
        slam_core.cpp
        orb_extractor.cpp
        slam_system.cpp dataset_loader.cpp
        ground_truth.cpp cv_utils.cpp orb_feature_matcher.cpp
        transfer_error.cpp transfer_error.h camera_motion_estimator.cpp camera_motion_estimator.h homogeneous_matrix.h homography_motion_estimator.cpp epipolar_constraint_motion_estimator.cpp homogeneous_matrix.cpp stamped_transformation.h eigen_utils.h eigen_utils.cpp epipolar_constraint_motion_estimator.h homography_motion_estimator.h projective_transformation.cpp projective_transformation.h plausible_transformation.cpp plausible_transformation.h viewer.cpp viewer.h settings.h bundle_adjustment.cpp bundle_adjustment.h octave_scales.cpp octave_scales.h optimizer.cpp optimizer.h ioc_factory.cpp ioc_factory.h undistorted_image_boundary.cpp undistorted_image_boundary.h bound.h cv_algorithms.cpp cv_algorithms.h map_initializer.cpp map_initializer.h states.h map.cpp map.h frame.cpp views/elements_view.h views/iterator.h views/elements_view.cpp views/iterator.cpp views/descriptor.h key_frame_graph.cpp key_frame_graph.h)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/third_party/DBoW2/lib/libDBoW2.so
        g2o
        )
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        ${PROJECT_SOURCE_DIR}/third_party
)