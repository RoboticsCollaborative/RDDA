# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /home/rc/clion-2019.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/rc/clion-2019.1.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rc/RDDA

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rc/RDDA/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/rdda_simpletest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rdda_simpletest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rdda_simpletest.dir/flags.make

CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o: CMakeFiles/rdda_simpletest.dir/flags.make
CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o: ../src/rdda/rdda_simpletest.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rc/RDDA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o   -c /home/rc/RDDA/src/rdda/rdda_simpletest.c

CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/rc/RDDA/src/rdda/rdda_simpletest.c > CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.i

CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/rc/RDDA/src/rdda/rdda_simpletest.c -o CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.s

# Object files for target rdda_simpletest
rdda_simpletest_OBJECTS = \
"CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o"

# External object files for target rdda_simpletest
rdda_simpletest_EXTERNAL_OBJECTS =

rdda_simpletest: CMakeFiles/rdda_simpletest.dir/src/rdda/rdda_simpletest.c.o
rdda_simpletest: CMakeFiles/rdda_simpletest.dir/build.make
rdda_simpletest: librdda_src.a
rdda_simpletest: CMakeFiles/rdda_simpletest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rc/RDDA/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable rdda_simpletest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rdda_simpletest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rdda_simpletest.dir/build: rdda_simpletest

.PHONY : CMakeFiles/rdda_simpletest.dir/build

CMakeFiles/rdda_simpletest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rdda_simpletest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rdda_simpletest.dir/clean

CMakeFiles/rdda_simpletest.dir/depend:
	cd /home/rc/RDDA/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rc/RDDA /home/rc/RDDA /home/rc/RDDA/cmake-build-debug /home/rc/RDDA/cmake-build-debug /home/rc/RDDA/cmake-build-debug/CMakeFiles/rdda_simpletest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rdda_simpletest.dir/depend

