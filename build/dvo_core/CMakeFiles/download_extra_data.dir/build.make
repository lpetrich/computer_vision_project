# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/laura/computer_vision/dvo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/laura/computer_vision/dvo/build

# Utility rule file for download_extra_data.

# Include the progress variables for this target.
include dvo_core/CMakeFiles/download_extra_data.dir/progress.make

download_extra_data: dvo_core/CMakeFiles/download_extra_data.dir/build.make
.PHONY : download_extra_data

# Rule to build all files generated by this target.
dvo_core/CMakeFiles/download_extra_data.dir/build: download_extra_data
.PHONY : dvo_core/CMakeFiles/download_extra_data.dir/build

dvo_core/CMakeFiles/download_extra_data.dir/clean:
	cd /home/laura/computer_vision/dvo/build/dvo_core && $(CMAKE_COMMAND) -P CMakeFiles/download_extra_data.dir/cmake_clean.cmake
.PHONY : dvo_core/CMakeFiles/download_extra_data.dir/clean

dvo_core/CMakeFiles/download_extra_data.dir/depend:
	cd /home/laura/computer_vision/dvo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laura/computer_vision/dvo /home/laura/computer_vision/dvo/dvo_core /home/laura/computer_vision/dvo/build /home/laura/computer_vision/dvo/build/dvo_core /home/laura/computer_vision/dvo/build/dvo_core/CMakeFiles/download_extra_data.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dvo_core/CMakeFiles/download_extra_data.dir/depend
