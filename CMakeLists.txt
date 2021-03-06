cmake_minimum_required(VERSION 3.5)
project(clean_slam VERSION 0.0.1)


IF (NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF ()
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set( CMAKE_VERBOSE_MAKEFILE on )
find_program(DISTCC_PROGRAM distcc)
if (DISTCC_PROGRAM)
    #    set(CMAKE_CXX_COMPILER_LAUNCHER "${DISTCC_PROGRAM}")
    message(STATUS "distcc: Enabled")
endif ()

set(USE_CCACHE ON CACHE BOOL "Use ccache to accelerate build")
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "ccache: Enabled")
endif ()
message(STATUS "DISTCC HOSTS:" $ENV{DISTCC_HOSTS})


MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Check C++17 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#the following define align is needed when we set CFLAGS -march=native
#add_compile_definitions(EIGEN_MAX_STATIC_ALIGN_BYTES=0) # workaround until eigen aligenment problem is solved
if (COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

add_definitions(-DFMT_HEADER_ONLY)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

add_subdirectory (src)
add_subdirectory(third_party/googletest)
add_subdirectory(third_party/g2o)
enable_testing()
add_subdirectory (test)



