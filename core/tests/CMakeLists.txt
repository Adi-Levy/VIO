add_executable(vio_core_estimator_default_state_test estimator_default_state_test.cpp)
target_link_libraries(vio_core_estimator_default_state_test PRIVATE vio_core)

add_executable(vio_core_estimator_valid_frame_test estimator_valid_frame_test.cpp)
target_link_libraries(vio_core_estimator_valid_frame_test PRIVATE vio_core)

add_executable(vio_core_estimator_invalid_frame_test estimator_invalid_frame_test.cpp)
target_link_libraries(vio_core_estimator_invalid_frame_test PRIVATE vio_core)

add_executable(vio_core_estimator_reset_test estimator_reset_test.cpp)
target_link_libraries(vio_core_estimator_reset_test PRIVATE vio_core)

add_executable(vio_core_feature_detector_test feature_detector_test.cpp)
target_include_directories(vio_core_feature_detector_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(vio_core_feature_detector_test PRIVATE vio_core)

add_executable(vio_core_klt_tracker_test klt_tracker_test.cpp)
target_include_directories(vio_core_klt_tracker_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_link_libraries(vio_core_klt_tracker_test PRIVATE vio_core)

add_test(
    NAME vio_core_estimator_default_state_test
    COMMAND vio_core_estimator_default_state_test
)

add_test(
    NAME vio_core_estimator_valid_frame_test
    COMMAND vio_core_estimator_valid_frame_test
)

add_test(
    NAME vio_core_estimator_invalid_frame_test
    COMMAND vio_core_estimator_invalid_frame_test
)

add_test(
    NAME vio_core_estimator_reset_test
    COMMAND vio_core_estimator_reset_test
)

add_test(
    NAME vio_core_feature_detector_test
    COMMAND vio_core_feature_detector_test
)

add_test(
    NAME vio_core_klt_tracker_test
    COMMAND vio_core_klt_tracker_test
)
