# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_SOURCE_DIR = /home/xingtian/catkin_kin/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xingtian/catkin_kin/build

# Utility rule file for clean_test_results_cv_bridge.

# Include any custom commands dependencies for this target.
include cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/compiler_depend.make

# Include the progress variables for this target.
include cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/progress.make

cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge:
	cd /home/xingtian/catkin_kin/build/cv_bridge/test && /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/remove_test_results.py /home/xingtian/catkin_kin/build/test_results/cv_bridge

clean_test_results_cv_bridge: cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge
clean_test_results_cv_bridge: cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/build.make
.PHONY : clean_test_results_cv_bridge

# Rule to build all files generated by this target.
cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/build: clean_test_results_cv_bridge
.PHONY : cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/build

cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/clean:
	cd /home/xingtian/catkin_kin/build/cv_bridge/test && $(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_cv_bridge.dir/cmake_clean.cmake
.PHONY : cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/clean

cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/depend:
	cd /home/xingtian/catkin_kin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xingtian/catkin_kin/src /home/xingtian/catkin_kin/src/cv_bridge/test /home/xingtian/catkin_kin/build /home/xingtian/catkin_kin/build/cv_bridge/test /home/xingtian/catkin_kin/build/cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cv_bridge/test/CMakeFiles/clean_test_results_cv_bridge.dir/depend

