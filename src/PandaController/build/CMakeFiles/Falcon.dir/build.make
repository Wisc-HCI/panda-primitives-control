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
CMAKE_SOURCE_DIR = /home/hcilab/Documents/PandaFCI/src/PandaController

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hcilab/Documents/PandaFCI/src/PandaController/build

# Include any dependencies generated for this target.
include CMakeFiles/Falcon.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Falcon.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Falcon.dir/flags.make

CMakeFiles/Falcon.dir/src/Falcon.cpp.o: CMakeFiles/Falcon.dir/flags.make
CMakeFiles/Falcon.dir/src/Falcon.cpp.o: ../src/Falcon.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hcilab/Documents/PandaFCI/src/PandaController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Falcon.dir/src/Falcon.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Falcon.dir/src/Falcon.cpp.o -c /home/hcilab/Documents/PandaFCI/src/PandaController/src/Falcon.cpp

CMakeFiles/Falcon.dir/src/Falcon.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Falcon.dir/src/Falcon.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hcilab/Documents/PandaFCI/src/PandaController/src/Falcon.cpp > CMakeFiles/Falcon.dir/src/Falcon.cpp.i

CMakeFiles/Falcon.dir/src/Falcon.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Falcon.dir/src/Falcon.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hcilab/Documents/PandaFCI/src/PandaController/src/Falcon.cpp -o CMakeFiles/Falcon.dir/src/Falcon.cpp.s

CMakeFiles/Falcon.dir/src/Falcon.cpp.o.requires:

.PHONY : CMakeFiles/Falcon.dir/src/Falcon.cpp.o.requires

CMakeFiles/Falcon.dir/src/Falcon.cpp.o.provides: CMakeFiles/Falcon.dir/src/Falcon.cpp.o.requires
	$(MAKE) -f CMakeFiles/Falcon.dir/build.make CMakeFiles/Falcon.dir/src/Falcon.cpp.o.provides.build
.PHONY : CMakeFiles/Falcon.dir/src/Falcon.cpp.o.provides

CMakeFiles/Falcon.dir/src/Falcon.cpp.o.provides.build: CMakeFiles/Falcon.dir/src/Falcon.cpp.o


# Object files for target Falcon
Falcon_OBJECTS = \
"CMakeFiles/Falcon.dir/src/Falcon.cpp.o"

# External object files for target Falcon
Falcon_EXTERNAL_OBJECTS =

Falcon: CMakeFiles/Falcon.dir/src/Falcon.cpp.o
Falcon: CMakeFiles/Falcon.dir/build.make
Falcon: /usr/local/lib/libnifalcon.so
Falcon: libPandaController.so
Falcon: libcommon.so
Falcon: CMakeFiles/Falcon.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hcilab/Documents/PandaFCI/src/PandaController/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Falcon"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Falcon.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Falcon.dir/build: Falcon

.PHONY : CMakeFiles/Falcon.dir/build

CMakeFiles/Falcon.dir/requires: CMakeFiles/Falcon.dir/src/Falcon.cpp.o.requires

.PHONY : CMakeFiles/Falcon.dir/requires

CMakeFiles/Falcon.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Falcon.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Falcon.dir/clean

CMakeFiles/Falcon.dir/depend:
	cd /home/hcilab/Documents/PandaFCI/src/PandaController/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hcilab/Documents/PandaFCI/src/PandaController /home/hcilab/Documents/PandaFCI/src/PandaController /home/hcilab/Documents/PandaFCI/src/PandaController/build /home/hcilab/Documents/PandaFCI/src/PandaController/build /home/hcilab/Documents/PandaFCI/src/PandaController/build/CMakeFiles/Falcon.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Falcon.dir/depend

