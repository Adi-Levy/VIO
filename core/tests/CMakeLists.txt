include(GoogleTest)

function(add_vio_gtest target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name}
        PRIVATE
            vio_core
            GTest::gtest
            GTest::gtest_main
    )
    gtest_discover_tests(${target_name})
endfunction()

function(add_vio_internal_gtest target_name source_file)
    add_vio_gtest(${target_name} ${source_file})
    target_include_directories(${target_name}
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../src
    )
endfunction()

add_vio_gtest(
    vio_core_estimator_test
    estimator_test.cpp
)
add_vio_internal_gtest(
    vio_core_feature_detector_test
    feature_detector_test.cpp
)
add_vio_internal_gtest(
    vio_core_klt_tracker_test
    klt_tracker_test.cpp
)
add_vio_internal_gtest(
    vio_core_essential_matrix_solver_test
    essential_matrix_solver_test.cpp
)
add_vio_internal_gtest(
    vio_core_pose_recovery_test
    pose_recovery_test.cpp
)
