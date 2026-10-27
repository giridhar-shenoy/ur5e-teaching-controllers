cmake_minimum_required(VERSION 3.16)
project(ur5e_teaching_controllers LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  generate_parameter_library
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dep} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)

generate_parameter_library(pid_ff_trajectory_controller_parameters
  config/pid_ff_trajectory_controller_parameters.yaml)
generate_parameter_library(gravity_comp_handguide_controller_parameters
  config/gravity_comp_handguide_controller_parameters.yaml)

# Dynamics wrapper: plain library (no ROS), unit-tested on its own.
add_library(ur5e_teaching_dynamics SHARED src/dynamics/pinocchio_model.cpp)
target_include_directories(ur5e_teaching_dynamics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(ur5e_teaching_dynamics PUBLIC pinocchio::pinocchio Eigen3::Eigen)

add_library(${PROJECT_NAME} SHARED
  src/effort_controller_base.cpp
  src/pid_ff_trajectory_controller.cpp
  src/gravity_comp_handguide_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ur5e_teaching_dynamics
  pid_ff_trajectory_controller_parameters
  gravity_comp_handguide_controller_parameters
  ${control_msgs_TARGETS}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${trajectory_msgs_TARGETS})

pluginlib_export_plugin_description_file(controller_interface ur5e_teaching_controllers.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME} ur5e_teaching_dynamics
    pid_ff_trajectory_controller_parameters gravity_comp_handguide_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(ur_description REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  # The tests run on the real UR5e description from ur_description (no copied numbers); the
  # mocked variant adds a mock_components/GenericSystem ros2_control block for the CM test.
  set(UR5E_TEST_URDF ${CMAKE_CURRENT_BINARY_DIR}/ur5e_test.urdf)
  add_custom_command(
    OUTPUT ${UR5E_TEST_URDF}
    COMMAND ${XACRO_EXECUTABLE} ${ur_description_DIR}/../urdf/ur_mocked.urdf.xacro
      ur_type:=ur5e name:=ur
      initial_positions_file:=${CMAKE_CURRENT_SOURCE_DIR}/test/initial_positions.yaml
      -o ${UR5E_TEST_URDF}
    # With calculate_dynamics the mock refuses effort-only claims ("has to have 'position',
    # 'velocity', or 'acceleration' interface"); the effort controllers need it off.
    COMMAND sed -i "s|calculate_dynamics\">true|calculate_dynamics\">false|" ${UR5E_TEST_URDF}
    DEPENDS test/initial_positions.yaml
    COMMENT "xacro: UR5e test URDF"
    VERBATIM)
  add_custom_target(ur5e_test_urdf ALL DEPENDS ${UR5E_TEST_URDF})

  ament_add_gtest(test_trapezoidal test/test_trapezoidal.cpp)
  target_include_directories(test_trapezoidal PRIVATE src)

  ament_add_gtest(test_control_laws test/test_control_laws.cpp)
  target_include_directories(test_control_laws PRIVATE src)

  ament_add_gtest(test_pinocchio_model test/test_pinocchio_model.cpp)
  target_link_libraries(test_pinocchio_model ur5e_teaching_dynamics)
  target_compile_definitions(test_pinocchio_model PRIVATE UR5E_TEST_URDF="${UR5E_TEST_URDF}")
  add_dependencies(test_pinocchio_model ur5e_test_urdf)

  ament_add_gmock(test_load_controllers test/test_load_controllers.cpp)
  target_link_libraries(test_load_controllers
    ur5e_teaching_dynamics
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets)
  target_compile_definitions(test_load_controllers PRIVATE UR5E_TEST_URDF="${UR5E_TEST_URDF}")
  add_dependencies(test_load_controllers ur5e_test_urdf ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3)
ament_package()
