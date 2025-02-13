cmake_minimum_required(VERSION 3.20)
project(status_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(/usr/local/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(status_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(sys_status_pub src/sys_status_pub.cpp)

target_link_libraries(sys_status_pub /usr/local/lib/libhwinfo_ram.so)
target_link_libraries(sys_status_pub /usr/local/lib/libhwinfo_cpu.so)

# target_include_directories(sys_status_pub PUBLIC include)
install(
    TARGETS sys_status_pub
    DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(sys_status_pub rclcpp status_interfaces)

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
