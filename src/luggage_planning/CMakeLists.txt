cmake_minimum_required(VERSION 3.8)
project(luggage_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_generator_node.py
  scripts/motion_planner_node.py
  scripts/vacuum_controller_node.py
  scripts/scene_manager_node.py
  scripts/hardware_pick_driver.py
  scripts/hardware_loading_console.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(luggage_planning_pytest test
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
endif()

ament_package()
