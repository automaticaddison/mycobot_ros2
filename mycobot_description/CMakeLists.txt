cmake_minimum_required(VERSION 3.8)
project(mycobot_description)

# Check if the compiler being used is GNU's C++ compiler (g++) or Clang.
# Add compiler flags for all targets that will be defined later in the
# CMakeLists file. These flags enable extra warnings to help catch
# potential issues in the code.
# Add options to the compilation process
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate and configure packages required by the project.
find_package(ament_cmake REQUIRED)
find_package(urdf_tutorial REQUIRED)

# Copy necessary files to designated locations in the project
install (
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

# Automates the process of setting up linting for the package, which
# is the process of running tools that analyze the code for potential
# errors, style issues, and other discrepancies that do not adhere to
# specified coding standards or best practices.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
