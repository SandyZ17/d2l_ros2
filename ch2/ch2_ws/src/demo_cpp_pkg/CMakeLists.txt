cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(cpp_node src/cpp_node.cpp)
add_executable(person_node src/person_node.cpp)
add_executable(learn_shared_ptr src/learn_shared_ptr.cpp)
add_executable(learn_lambda src/learn_lambda.cpp)
add_executable(learn_functional src/learn_functional.cpp)
add_executable(learn_thread src/learn_thread.cpp)

target_include_directories(learn_thread PUBLIC /usr/include)

install(
    TARGETS cpp_node person_node learn_shared_ptr learn_lambda learn_functional learn_thread
    DESTINATION lib/${PROJECT_NAME}
)

# message(STATUS ${rclcpp_INCLUDE_DIRS})
# message(STATUS ${rclcpp_LIBRARIES})

# target_include_directories(cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})
# target_link_libraries(cpp_node ${rclcpp_LIBRARIES})
ament_target_dependencies(cpp_node rclcpp)
ament_target_dependencies(person_node rclcpp)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)

    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
