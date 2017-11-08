project(TEST)

add_executable(Test tests.cpp
  ../src/tools.cpp
  ../src/vehicle.cpp
  ../src/spline.h
  ../src/const.h
  ../src/costfunctions.cpp
  ../src/sensorfusion.cpp)

target_link_libraries(Test z ssl uv uWS)
