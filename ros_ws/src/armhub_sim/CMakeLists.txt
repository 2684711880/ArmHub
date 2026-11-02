cmake_minimum_required(VERSION 3.16)
project(armhub_sim)
find_package(ament_cmake REQUIRED)
get_filename_component(ARMHUB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${ARMHUB_ROOT}/robots" "${ARMHUB_ROOT}/vendor" DESTINATION share/${PROJECT_NAME}
    PATTERN "sdk" EXCLUDE)
ament_package()
