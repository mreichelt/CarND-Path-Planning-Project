project(TEST)

add_executable(Test tests.cpp ../src/tools.cpp ../src/Vehicle.cpp)

target_link_libraries(Test z ssl uv uWS)
