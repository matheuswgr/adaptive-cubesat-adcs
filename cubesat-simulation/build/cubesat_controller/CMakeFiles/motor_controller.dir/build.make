# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller

# Include any dependencies generated for this target.
include CMakeFiles/motor_controller.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/motor_controller.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/motor_controller.dir/flags.make

CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o: CMakeFiles/motor_controller.dir/flags.make
CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o: /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller/src/motor_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o -c /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller/src/motor_controller.cpp

CMakeFiles/motor_controller.dir/src/motor_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/motor_controller.dir/src/motor_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller/src/motor_controller.cpp > CMakeFiles/motor_controller.dir/src/motor_controller.cpp.i

CMakeFiles/motor_controller.dir/src/motor_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/motor_controller.dir/src/motor_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller/src/motor_controller.cpp -o CMakeFiles/motor_controller.dir/src/motor_controller.cpp.s

# Object files for target motor_controller
motor_controller_OBJECTS = \
"CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o"

# External object files for target motor_controller
motor_controller_EXTERNAL_OBJECTS =

motor_controller: CMakeFiles/motor_controller.dir/src/motor_controller.cpp.o
motor_controller: CMakeFiles/motor_controller.dir/build.make
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librclcpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_generator_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librmw_implementation.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librmw.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcutils.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_logging_noop.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_generator_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosgraph_msgs__rosidl_typesupport_fastrtps_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librcl_yaml_param_parser.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_generator_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_fastrtps_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosidl_typesupport_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosidl_typesupport_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosidl_generator_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosidl_typesupport_introspection_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/librosidl_typesupport_introspection_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_generator_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_fastrtps_c.so
motor_controller: /home/matheuswagner/ros2_dashing/ros2-linux/lib/libstd_msgs__rosidl_typesupport_fastrtps_cpp.so
motor_controller: CMakeFiles/motor_controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable motor_controller"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/motor_controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/motor_controller.dir/build: motor_controller

.PHONY : CMakeFiles/motor_controller.dir/build

CMakeFiles/motor_controller.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/motor_controller.dir/cmake_clean.cmake
.PHONY : CMakeFiles/motor_controller.dir/clean

CMakeFiles/motor_controller.dir/depend:
	cd /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/src/cubesat_controller /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller /home/matheuswagner/repos/adaptive-cubesat-adcs/cubesat-simulation/build/cubesat_controller/CMakeFiles/motor_controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/motor_controller.dir/depend

