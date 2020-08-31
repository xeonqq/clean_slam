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
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} graph.h)

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