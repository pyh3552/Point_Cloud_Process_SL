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
CMAKE_COMMAND = /home/pyh/下载/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/pyh/下载/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pyh/文档/GitHub/Point_Cloud_Process_SL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug

# Include any dependencies generated for this target.
include 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/depend.make

# Include the progress variables for this target.
include 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/progress.make

# Include the compile flags for this target's objects.
include 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/flags.make

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/flags.make
01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o: ../01-Intro_and_basic_algorithm/code/pca_normal_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o -c /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/pca_normal_main.cpp

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.i"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/pca_normal_main.cpp > CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.i

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.s"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/pca_normal_main.cpp -o CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.s

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/flags.make
01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o: ../01-Intro_and_basic_algorithm/code/include/pca_normal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o -c /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/include/pca_normal.cpp

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.i"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/include/pca_normal.cpp > CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.i

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.s"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm/code/include/pca_normal.cpp -o CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.s

# Object files for target voxel_filter
voxel_filter_OBJECTS = \
"CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o" \
"CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o"

# External object files for target voxel_filter
voxel_filter_EXTERNAL_OBJECTS =

01-Intro_and_basic_algorithm/voxel_filter: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/pca_normal_main.cpp.o
01-Intro_and_basic_algorithm/voxel_filter: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/code/include/pca_normal.cpp.o
01-Intro_and_basic_algorithm/voxel_filter: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/build.make
01-Intro_and_basic_algorithm/voxel_filter: 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable voxel_filter"
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/voxel_filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/build: 01-Intro_and_basic_algorithm/voxel_filter

.PHONY : 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/build

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/clean:
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm && $(CMAKE_COMMAND) -P CMakeFiles/voxel_filter.dir/cmake_clean.cmake
.PHONY : 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/clean

01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/depend:
	cd /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pyh/文档/GitHub/Point_Cloud_Process_SL /home/pyh/文档/GitHub/Point_Cloud_Process_SL/01-Intro_and_basic_algorithm /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm /home/pyh/文档/GitHub/Point_Cloud_Process_SL/cmake-build-debug/01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : 01-Intro_and_basic_algorithm/CMakeFiles/voxel_filter.dir/depend

