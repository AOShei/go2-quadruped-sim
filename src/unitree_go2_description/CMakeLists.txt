cmake_minimum_required(VERSION 3.8)
project(unitree_go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    urdf
    xacro
    config
    meshes
    rviz
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # This validates your xacro files during testing
  find_package(xacro REQUIRED)
  
  # Validate that xacro processing works and produces valid URDF
  add_custom_target(validate_xacro
    COMMAND ${CMAKE_COMMAND} -E echo "Validating xacro -> URDF conversion..."
    COMMAND xacro ${CMAKE_CURRENT_SOURCE_DIR}/xacro/unitree_go2.xacro > /tmp/unitree_go2_test.urdf
    COMMAND check_urdf /tmp/unitree_go2_test.urdf
    COMMAND ${CMAKE_COMMAND} -E echo "✅ URDF validation successful!"
    COMMENT "Validating xacro files produce valid URDF"
    VERBATIM
  )
  
  # You can run this manually with: colcon build --packages-select unitree_go2_description --cmake-target validate_xacro
endif()

ament_package()
