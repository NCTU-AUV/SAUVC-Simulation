cmake_minimum_required(VERSION 3.8)
project(sauvc_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# --- Install non-code assets ---
install(DIRECTORY models/
        DESTINATION share/${PROJECT_NAME}/models)

install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

# --- Environment hook to expose assets to Gazebo ---
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/${PROJECT_NAME}_resource_path.dsv.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_resource_path.dsv
  @ONLY
)
ament_environment_hooks(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_resource_path.dsv)

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
