# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rtrk/myROSworkspace/src/bachelor

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rtrk/myROSworkspace/src/bachelor/build

# Utility rule file for clean_test_results_bachelor.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_bachelor.dir/progress.make

CMakeFiles/clean_test_results_bachelor:
	/usr/bin/python2 /opt/ros/melodic/share/catkin/cmake/test/remove_test_results.py /home/rtrk/myROSworkspace/src/bachelor/build/test_results/bachelor

clean_test_results_bachelor: CMakeFiles/clean_test_results_bachelor
clean_test_results_bachelor: CMakeFiles/clean_test_results_bachelor.dir/build.make

.PHONY : clean_test_results_bachelor

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_bachelor.dir/build: clean_test_results_bachelor

.PHONY : CMakeFiles/clean_test_results_bachelor.dir/build

CMakeFiles/clean_test_results_bachelor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_bachelor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_bachelor.dir/clean

CMakeFiles/clean_test_results_bachelor.dir/depend:
	cd /home/rtrk/myROSworkspace/src/bachelor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rtrk/myROSworkspace/src/bachelor /home/rtrk/myROSworkspace/src/bachelor /home/rtrk/myROSworkspace/src/bachelor/build /home/rtrk/myROSworkspace/src/bachelor/build /home/rtrk/myROSworkspace/src/bachelor/build/CMakeFiles/clean_test_results_bachelor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_bachelor.dir/depend

