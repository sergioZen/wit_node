add_executable(wit_node wit_node.cpp)
add_dependencies(wit_node wit_ros)
target_link_libraries(wit_node wit_ros)
