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
CMAKE_SOURCE_DIR = /root/catkin_ws/src/Decodertest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/catkin_ws/src/Decodertest/bulid

# Include any dependencies generated for this target.
include CMakeFiles/decodertest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/decodertest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/decodertest.dir/flags.make

CMakeFiles/decodertest.dir/main.cpp.o: CMakeFiles/decodertest.dir/flags.make
CMakeFiles/decodertest.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/catkin_ws/src/Decodertest/bulid/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/decodertest.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/decodertest.dir/main.cpp.o -c /root/catkin_ws/src/Decodertest/main.cpp

CMakeFiles/decodertest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/decodertest.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/catkin_ws/src/Decodertest/main.cpp > CMakeFiles/decodertest.dir/main.cpp.i

CMakeFiles/decodertest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/decodertest.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/catkin_ws/src/Decodertest/main.cpp -o CMakeFiles/decodertest.dir/main.cpp.s

CMakeFiles/decodertest.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/decodertest.dir/main.cpp.o.requires

CMakeFiles/decodertest.dir/main.cpp.o.provides: CMakeFiles/decodertest.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/decodertest.dir/build.make CMakeFiles/decodertest.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/decodertest.dir/main.cpp.o.provides

CMakeFiles/decodertest.dir/main.cpp.o.provides.build: CMakeFiles/decodertest.dir/main.cpp.o


# Object files for target decodertest
decodertest_OBJECTS = \
"CMakeFiles/decodertest.dir/main.cpp.o"

# External object files for target decodertest
decodertest_EXTERNAL_OBJECTS =

decodertest: CMakeFiles/decodertest.dir/main.cpp.o
decodertest: CMakeFiles/decodertest.dir/build.make
decodertest: CMakeFiles/decodertest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/catkin_ws/src/Decodertest/bulid/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable decodertest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/decodertest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/decodertest.dir/build: decodertest

.PHONY : CMakeFiles/decodertest.dir/build

CMakeFiles/decodertest.dir/requires: CMakeFiles/decodertest.dir/main.cpp.o.requires

.PHONY : CMakeFiles/decodertest.dir/requires

CMakeFiles/decodertest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/decodertest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/decodertest.dir/clean

CMakeFiles/decodertest.dir/depend:
	cd /root/catkin_ws/src/Decodertest/bulid && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/catkin_ws/src/Decodertest /root/catkin_ws/src/Decodertest /root/catkin_ws/src/Decodertest/bulid /root/catkin_ws/src/Decodertest/bulid /root/catkin_ws/src/Decodertest/bulid/CMakeFiles/decodertest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/decodertest.dir/depend
