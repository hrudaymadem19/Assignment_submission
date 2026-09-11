cmake_minimum_required(VERSION 3.8)
project(slam_robot_test)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/auto_drive.py
    scripts/noise_injector.py
    scripts/ekf_estimator.py
    scripts/mapping_diagnostics.py
    scripts/wall_controller.py
    scripts/world_visualizer.py
    scripts/plot_summary.py
    scripts/scan_matcher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    urdf
    config
    worlds
    rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
