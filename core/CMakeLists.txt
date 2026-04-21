set(VIO_CORE_SOURCES
    src/empty.cpp
    src/estimator/estimator.cpp
    src/frontend/frontend.cpp
    src/imu/imu.cpp
    src/initialization/initialization.cpp
)

add_library(vio_core ${VIO_CORE_SOURCES})

target_include_directories(vio_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

enable_testing()
add_subdirectory(tests)
