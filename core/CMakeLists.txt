set(VIO_CORE_SOURCES
    src/estimator/vo_estimator.cpp
    src/estimator/vo_state_manager.cpp
    src/visual_processing/feature_detector.cpp
    src/visual_processing/klt_tracker.cpp
    src/motion_geometry/essential_matrix_solver.cpp
    src/motion_geometry/pose_recovery.cpp
    src/motion_geometry/triangulator.cpp
    src/diagnostics/diagnostics_builder.cpp
    src/utils/image_utils.cpp
    src/utils/validation.cpp
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video)
find_package(GTest CONFIG QUIET)
if(NOT GTest_FOUND)
    find_package(GTest QUIET)
endif()
if(NOT GTest_FOUND)
    message(FATAL_ERROR
        "GTest was not found. On Ubuntu 22.04 install it with: sudo apt install libgtest-dev"
    )
endif()

add_library(vio_core ${VIO_CORE_SOURCES})

target_include_directories(vio_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(vio_core
    PUBLIC
        Eigen3::Eigen
        opencv_core
        opencv_imgproc
        opencv_video
)

add_subdirectory(tests)
