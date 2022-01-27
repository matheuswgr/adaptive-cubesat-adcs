# CMake generated Testfile for 
# Source directory: /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo
# Build directory: /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/copyright.xunit.xml" "--package-name" "cubesat_gazebo" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/ament_copyright/copyright.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_copyright" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo")
add_test(flake8 "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/flake8.xunit.xml" "--package-name" "cubesat_gazebo" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/ament_flake8/flake8.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_flake8" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo")
add_test(lint_cmake "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/lint_cmake.xunit.xml" "--package-name" "cubesat_gazebo" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/ament_lint_cmake/lint_cmake.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_lint_cmake" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo")
add_test(pep257 "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/pep257.xunit.xml" "--package-name" "cubesat_gazebo" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/ament_pep257/pep257.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_pep257" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo")
add_test(xmllint "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/xmllint.xunit.xml" "--package-name" "cubesat_gazebo" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/ament_xmllint/xmllint.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_xmllint" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_gazebo/test_results/cubesat_gazebo/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_gazebo")
