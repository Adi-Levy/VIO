set(VIO_CORE_SOURCES
    src/empty.cpp
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
find_package(OpenCV REQUIRED COMPONENTS core)

add_library(vio_core ${VIO_CORE_SOURCES})

target_include_directories(vio_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(vio_core
    PUBLIC
        Eigen3::Eigen
        opencv_core
)

add_subdirectory(tests)
