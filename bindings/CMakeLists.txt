pybind11_add_module(pyslam slam_py.cpp)
target_link_libraries(pyslam PRIVATE slam_core)
