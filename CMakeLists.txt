cmake_minimum_required(VERSION 3.8)
project(gnss_reset)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module)
find_package(Eigen3)

ament_auto_find_build_dependencies()

ament_auto_add_executable(embed_gnss2map
  src/embed_gnss2map.cpp
)

ament_auto_add_executable(gnss_reset
  src/gnss_reset.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
