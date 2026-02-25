cmake_minimum_required(VERSION 3.8)
project(lunabot_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.flake8")
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  worlds
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_sim_topics.py
  scripts/check_motion_response.py
  scripts/smoke_test_classic.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Keep model path available when running Gazebo Classic locally.
set(ENV{GAZEBO_MODEL_PATH} "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models:$ENV{GAZEBO_MODEL_PATH}")

ament_package()
