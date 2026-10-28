cmake_minimum_required(VERSION 3.8)
project(nero_dual_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/joint_bridge scripts/pointing_server scripts/pointing_camera scripts/pointing_passive_arm scripts/cabinet_scene DESTINATION lib/${PROJECT_NAME})
get_filename_component(NERO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
install(DIRECTORY "${NERO_ROOT}/third_party/open_nero/open_nero_description/urdf"
                  "${NERO_ROOT}/third_party/open_nero/open_nero_description/meshes"
        DESTINATION share/${PROJECT_NAME}/open_nero_description)
ament_package()
