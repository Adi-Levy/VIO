add_executable(vio_core_smoke_test smoke_test.cpp)
target_link_libraries(vio_core_smoke_test PRIVATE vio_core)

add_test(NAME vio_core_smoke_test COMMAND vio_core_smoke_test)
