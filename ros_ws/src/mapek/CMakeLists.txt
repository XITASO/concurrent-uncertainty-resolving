cmake_minimum_required(VERSION 3.8)
project(mapek)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#SET(CMAKE_CXX_FLAGS_DEBUG "-O2")
#SET(CMAKE_C_FLAGS_DEBUG "-O2")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(system_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(system_modes REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Source files for mapek library
file(GLOB MAPEK_SRC "src/*.cpp" "src/*/*.cpp" "src/*/*/*.cpp")

# Create mapek library
add_library(mapek_lib SHARED ${MAPEK_SRC})

add_executable(strategy_selector src/main.cpp src/planning.cpp src/util/system_analyzer.cpp)

# Include directories for mapek library
target_include_directories(mapek_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(strategy_selector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Specify export targets
ament_export_targets(export_mapek HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake rclcpp system_interfaces system_modes)

# Specify package dependencies for the library
ament_target_dependencies(mapek_lib
  rclcpp
  system_interfaces
  system_modes
)

ament_target_dependencies(strategy_selector rclcpp system_interfaces)

target_link_libraries(strategy_selector mapek_lib)

# Install library and include headers
install(TARGETS mapek_lib
  EXPORT export_mapek
  LIBRARY DESTINATION lib
)

install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS strategy_selector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()

  find_package(GTest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB_RECURSE TESTFILES tests/*.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test
    system_interfaces
    rclcpp
    rcl_interfaces
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test mapek_lib)

  message(${CMAKE_CURRENT_SOURCE_DIR})

  set_tests_properties(${Tests} PROPERTIES TIMEOUT 3600)
  set_tests_properties(
    ${PROJECT_NAME}_test PROPERTIES 
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

endif()

ament_package()
