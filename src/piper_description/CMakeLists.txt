cmake_minimum_required(VERSION 3.5)
project(piper_description)

# 找到ament_cmake来构建这个包
find_package(ament_cmake REQUIRED)

# 安装文件夹
install(DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/piper_joint_preview_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_robot_description
    test/test_robot_description.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

# 包的描述
ament_package()
