# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/user/intel_tera_mavlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/intel_tera_mavlink/build

# Utility rule file for doxygen.

# Include the progress variables for this target.
include serial/CMakeFiles/doxygen.dir/progress.make

doxygen: serial/CMakeFiles/doxygen.dir/build.make

.PHONY : doxygen

# Rule to build all files generated by this target.
serial/CMakeFiles/doxygen.dir/build: doxygen

.PHONY : serial/CMakeFiles/doxygen.dir/build

serial/CMakeFiles/doxygen.dir/clean:
	cd /home/user/intel_tera_mavlink/build/serial && $(CMAKE_COMMAND) -P CMakeFiles/doxygen.dir/cmake_clean.cmake
.PHONY : serial/CMakeFiles/doxygen.dir/clean

serial/CMakeFiles/doxygen.dir/depend:
	cd /home/user/intel_tera_mavlink/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/intel_tera_mavlink /home/user/intel_tera_mavlink/serial /home/user/intel_tera_mavlink/build /home/user/intel_tera_mavlink/build/serial /home/user/intel_tera_mavlink/build/serial/CMakeFiles/doxygen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial/CMakeFiles/doxygen.dir/depend

