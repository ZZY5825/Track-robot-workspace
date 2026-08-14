cmake_minimum_required(VERSION 3.5)
project(track_robot_semantic_search_rviz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(track_robot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(query_session STATIC
  src/query_session.cpp
)
set_target_properties(query_session PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(query_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(query_session
  Qt5::Core
  nlohmann_json::nlohmann_json
)

add_library(active_search_session STATIC
  src/active_search_session.cpp
)
target_include_directories(active_search_session PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/semantic_search_panel.hpp
  src/semantic_search_panel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  diagnostic_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rviz_common
  std_msgs
  std_srvs
  track_robot_interfaces
)
target_link_libraries(${PROJECT_NAME}
  query_session
  active_search_session
  Qt5::Core
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(
  rviz_common
  plugin_description.xml
)

if(BUILD_TESTING)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_query_session test/test_query_session.cpp)
  target_include_directories(test_query_session PRIVATE include)
  target_link_libraries(
    test_query_session
    query_session
    active_search_session
    Qt5::Core
    nlohmann_json::nlohmann_json
  )

  ament_add_gtest(test_active_search_session test/test_active_search_session.cpp)
  target_include_directories(test_active_search_session PRIVATE include)
  target_link_libraries(test_active_search_session active_search_session)

  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_plugin_contract
    test/test_plugin_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  diagnostic_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rviz_common
  std_msgs
  std_srvs
  track_robot_interfaces
)

ament_package()
