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

# Include any dependencies generated for this target.
include /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/depend.make

# Include the progress variables for this target.
include /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/progress.make

# Include the compile flags for this target's objects.
include /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/flags.make

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/flags.make
/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o: /usr/src/gtest/src/gtest-all.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/laura/computer_vision/dvo/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o"
	cd /home/laura/computer_vision/dvo/bin/gtest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gtest.dir/src/gtest-all.cc.o -c /usr/src/gtest/src/gtest-all.cc

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest.dir/src/gtest-all.cc.i"
	cd /home/laura/computer_vision/dvo/bin/gtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /usr/src/gtest/src/gtest-all.cc > CMakeFiles/gtest.dir/src/gtest-all.cc.i

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest.dir/src/gtest-all.cc.s"
	cd /home/laura/computer_vision/dvo/bin/gtest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /usr/src/gtest/src/gtest-all.cc -o CMakeFiles/gtest.dir/src/gtest-all.cc.s

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires:
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires
	$(MAKE) -f /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/build.make /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.provides.build: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o

# Object files for target gtest
gtest_OBJECTS = \
"CMakeFiles/gtest.dir/src/gtest-all.cc.o"

# External object files for target gtest
gtest_EXTERNAL_OBJECTS =

/home/laura/computer_vision/dvo/bin/libgtest.so: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o
/home/laura/computer_vision/dvo/bin/libgtest.so: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/build.make
/home/laura/computer_vision/dvo/bin/libgtest.so: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/laura/computer_vision/dvo/bin/libgtest.so"
	cd /home/laura/computer_vision/dvo/bin/gtest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/build: /home/laura/computer_vision/dvo/bin/libgtest.so
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/build

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/requires: /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/src/gtest-all.cc.o.requires
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/requires

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/clean:
	cd /home/laura/computer_vision/dvo/bin/gtest && $(CMAKE_COMMAND) -P CMakeFiles/gtest.dir/cmake_clean.cmake
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/clean

/home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/depend:
	cd /home/laura/computer_vision/dvo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/laura/computer_vision/dvo /usr/src/gtest /home/laura/computer_vision/dvo/build /home/laura/computer_vision/dvo/bin/gtest /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : /home/laura/computer_vision/dvo/bin/gtest/CMakeFiles/gtest.dir/depend

