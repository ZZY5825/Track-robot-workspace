cmake_minimum_required(VERSION 3.5)
project(track_robot_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# 后面你会放这些目录：
#   launch/
#   config/
#   rviz/
# 先把 install 规则写好（即使目录暂时为空也没问题）
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/semantic_search_ctl
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The workspace carries a Python 3.8 pytest plugin incompatible with Foxy's
  # pytest 4.6; keep unrelated entry-point plugins out of this unit test.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_control_config
    test/test_control_config.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_readiness
    test/test_readiness.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_process_control
    test/test_process_control.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_control_cli
    test/test_control_cli.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_live_report
    test/test_live_report.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_quick_start_doc
    test/test_quick_start_doc.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
