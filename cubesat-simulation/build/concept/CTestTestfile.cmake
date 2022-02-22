# CMake generated Testfile for 
# Source directory: /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept
# Build directory: /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(copyright "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/copyright.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_copyright/copyright.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_copyright" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/copyright.xunit.xml")
set_tests_properties(copyright PROPERTIES  LABELS "copyright;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_copyright/cmake/ament_copyright.cmake;41;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;31;ament_copyright;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_copyright/cmake/ament_cmake_copyright_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
add_test(cppcheck "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/cppcheck.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_cppcheck/cppcheck.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_cppcheck" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/cppcheck.xunit.xml")
set_tests_properties(cppcheck PROPERTIES  LABELS "cppcheck;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cppcheck/cmake/ament_cppcheck.cmake;55;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;67;ament_cppcheck;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cppcheck/cmake/ament_cmake_cppcheck_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
add_test(cpplint "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/cpplint.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_cpplint/cpplint.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_cpplint" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/cpplint.xunit.xml")
set_tests_properties(cpplint PROPERTIES  LABELS "cpplint;linter" TIMEOUT "120" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cpplint/cmake/ament_cpplint.cmake;63;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;27;ament_cpplint;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_cpplint/cmake/ament_cmake_cpplint_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
add_test(lint_cmake "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/lint_cmake.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_lint_cmake/lint_cmake.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_lint_cmake" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;41;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
add_test(uncrustify "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/uncrustify.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_uncrustify/uncrustify.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_uncrustify" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/uncrustify.xunit.xml")
set_tests_properties(uncrustify PROPERTIES  LABELS "uncrustify;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_uncrustify/cmake/ament_uncrustify.cmake;47;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;27;ament_uncrustify;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_uncrustify/cmake/ament_cmake_uncrustify_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
add_test(xmllint "/usr/bin/python3" "-u" "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/run_test.py" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/xmllint.xunit.xml" "--package-name" "concept" "--output-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/ament_xmllint/xmllint.txt" "--command" "/home/matheuswagner/ros2_dashing/ros2-linux/bin/ament_xmllint" "--xunit-file" "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/concept/test_results/concept/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept" _BACKTRACE_TRIPLES "/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_test/cmake/ament_add_test.cmake;118;add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/matheuswagner/ros2_dashing/ros2-linux/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;41;ament_package;/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/concept/CMakeLists.txt;0;")
