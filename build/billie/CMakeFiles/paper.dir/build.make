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
CMAKE_SOURCE_DIR = /home/mfikih15/Documents/skripsi_billie/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mfikih15/Documents/skripsi_billie/build

# Include any dependencies generated for this target.
include billie/CMakeFiles/paper.dir/depend.make

# Include the progress variables for this target.
include billie/CMakeFiles/paper.dir/progress.make

# Include the compile flags for this target's objects.
include billie/CMakeFiles/paper.dir/flags.make

billie/CMakeFiles/paper.dir/src/paper.cpp.o: billie/CMakeFiles/paper.dir/flags.make
billie/CMakeFiles/paper.dir/src/paper.cpp.o: /home/mfikih15/Documents/skripsi_billie/src/billie/src/paper.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mfikih15/Documents/skripsi_billie/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object billie/CMakeFiles/paper.dir/src/paper.cpp.o"
	cd /home/mfikih15/Documents/skripsi_billie/build/billie && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/paper.dir/src/paper.cpp.o -c /home/mfikih15/Documents/skripsi_billie/src/billie/src/paper.cpp

billie/CMakeFiles/paper.dir/src/paper.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/paper.dir/src/paper.cpp.i"
	cd /home/mfikih15/Documents/skripsi_billie/build/billie && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mfikih15/Documents/skripsi_billie/src/billie/src/paper.cpp > CMakeFiles/paper.dir/src/paper.cpp.i

billie/CMakeFiles/paper.dir/src/paper.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/paper.dir/src/paper.cpp.s"
	cd /home/mfikih15/Documents/skripsi_billie/build/billie && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mfikih15/Documents/skripsi_billie/src/billie/src/paper.cpp -o CMakeFiles/paper.dir/src/paper.cpp.s

billie/CMakeFiles/paper.dir/src/paper.cpp.o.requires:

.PHONY : billie/CMakeFiles/paper.dir/src/paper.cpp.o.requires

billie/CMakeFiles/paper.dir/src/paper.cpp.o.provides: billie/CMakeFiles/paper.dir/src/paper.cpp.o.requires
	$(MAKE) -f billie/CMakeFiles/paper.dir/build.make billie/CMakeFiles/paper.dir/src/paper.cpp.o.provides.build
.PHONY : billie/CMakeFiles/paper.dir/src/paper.cpp.o.provides

billie/CMakeFiles/paper.dir/src/paper.cpp.o.provides.build: billie/CMakeFiles/paper.dir/src/paper.cpp.o


# Object files for target paper
paper_OBJECTS = \
"CMakeFiles/paper.dir/src/paper.cpp.o"

# External object files for target paper
paper_EXTERNAL_OBJECTS =

/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: billie/CMakeFiles/paper.dir/src/paper.cpp.o
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: billie/CMakeFiles/paper.dir/build.make
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libmavros.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libGeographic.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libclass_loader.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/libPocoFoundation.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libdl.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libroslib.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/librospack.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libtf2_ros.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libactionlib.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libmessage_filters.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libroscpp.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/librosconsole.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libtf2.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libmavconn.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libeigen_conversions.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/liborocos-kdl.so.1.3.0
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/librostime.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /opt/ros/kinetic/lib/libcpp_common.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper: billie/CMakeFiles/paper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mfikih15/Documents/skripsi_billie/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper"
	cd /home/mfikih15/Documents/skripsi_billie/build/billie && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/paper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
billie/CMakeFiles/paper.dir/build: /home/mfikih15/Documents/skripsi_billie/devel/lib/billie/paper

.PHONY : billie/CMakeFiles/paper.dir/build

billie/CMakeFiles/paper.dir/requires: billie/CMakeFiles/paper.dir/src/paper.cpp.o.requires

.PHONY : billie/CMakeFiles/paper.dir/requires

billie/CMakeFiles/paper.dir/clean:
	cd /home/mfikih15/Documents/skripsi_billie/build/billie && $(CMAKE_COMMAND) -P CMakeFiles/paper.dir/cmake_clean.cmake
.PHONY : billie/CMakeFiles/paper.dir/clean

billie/CMakeFiles/paper.dir/depend:
	cd /home/mfikih15/Documents/skripsi_billie/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mfikih15/Documents/skripsi_billie/src /home/mfikih15/Documents/skripsi_billie/src/billie /home/mfikih15/Documents/skripsi_billie/build /home/mfikih15/Documents/skripsi_billie/build/billie /home/mfikih15/Documents/skripsi_billie/build/billie/CMakeFiles/paper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : billie/CMakeFiles/paper.dir/depend

