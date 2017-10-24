project(TEST)

add_executable(Test tests.cpp ../src/tools.cpp ../src/Vehicle.cpp ../src/spline.h)

target_link_libraries(Test z ssl uv uWS)
