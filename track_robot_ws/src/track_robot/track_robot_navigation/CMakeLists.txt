cmake_minimum_required(VERSION 3.5)
project(track_robot_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/search_motion_adapter
  scripts/semantic_navigation_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_runtime_modes
    test/test_runtime_modes.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_nav2_config_contract
    test/test_nav2_config_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_phase5a_nav2_config_contract
    test/test_phase5a_nav2_config_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_phase5a_launch_contract
    test/test_phase5a_launch_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_dynamic_obstacle_sources
    test/test_dynamic_obstacle_sources.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_semantic_goal_policy
    test/test_semantic_goal_policy.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_semantic_authorization
    test/test_semantic_authorization.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_search_motion_adapter
    test/test_search_motion_adapter.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_static_target_mission
    test/test_static_target_mission.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_physical_recovery
    test/test_physical_recovery.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_physical_recovery_supervisor
    test/test_physical_recovery_supervisor.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
