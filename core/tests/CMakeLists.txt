add_executable(vio_core_estimator_default_state_test estimator_default_state_test.cpp)
target_link_libraries(vio_core_estimator_default_state_test PRIVATE vio_core)

add_executable(vio_core_estimator_valid_frame_test estimator_valid_frame_test.cpp)
target_link_libraries(vio_core_estimator_valid_frame_test PRIVATE vio_core)

add_executable(vio_core_estimator_invalid_frame_test estimator_invalid_frame_test.cpp)
target_link_libraries(vio_core_estimator_invalid_frame_test PRIVATE vio_core)

add_executable(vio_core_estimator_reset_test estimator_reset_test.cpp)
target_link_libraries(vio_core_estimator_reset_test PRIVATE vio_core)

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
