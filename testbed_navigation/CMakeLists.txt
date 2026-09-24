cmake_minimum_required(VERSION 3.8)
project(testbed_navigation)

find_package(ament_cmake REQUIRED)

option(TESTBED_RUNTIME_TESTS "Register the runtime navigation tests, which need Gazebo" OFF)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(static test/static TIMEOUT 300)

  if(TESTBED_RUNTIME_TESTS)
    ament_add_pytest_test(runtime test/runtime TIMEOUT 1800)
  endif()
endif()

ament_package()
