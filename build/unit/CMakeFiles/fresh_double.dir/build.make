# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rokon/renew/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rokon/renew/build

# Include any dependencies generated for this target.
include unit/CMakeFiles/fresh_double.dir/depend.make

# Include the progress variables for this target.
include unit/CMakeFiles/fresh_double.dir/progress.make

# Include the compile flags for this target's objects.
include unit/CMakeFiles/fresh_double.dir/flags.make

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o: unit/CMakeFiles/fresh_double.dir/flags.make
unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o: /home/rokon/renew/src/unit/fresh_double.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rokon/renew/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o"
	cd /home/rokon/renew/build/unit && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/fresh_double.dir/fresh_double.cpp.o -c /home/rokon/renew/src/unit/fresh_double.cpp

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fresh_double.dir/fresh_double.cpp.i"
	cd /home/rokon/renew/build/unit && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rokon/renew/src/unit/fresh_double.cpp > CMakeFiles/fresh_double.dir/fresh_double.cpp.i

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fresh_double.dir/fresh_double.cpp.s"
	cd /home/rokon/renew/build/unit && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rokon/renew/src/unit/fresh_double.cpp -o CMakeFiles/fresh_double.dir/fresh_double.cpp.s

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.requires:
.PHONY : unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.requires

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.provides: unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.requires
	$(MAKE) -f unit/CMakeFiles/fresh_double.dir/build.make unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.provides.build
.PHONY : unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.provides

unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.provides.build: unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o

# Object files for target fresh_double
fresh_double_OBJECTS = \
"CMakeFiles/fresh_double.dir/fresh_double.cpp.o"

# External object files for target fresh_double
fresh_double_EXTERNAL_OBJECTS =

/home/rokon/renew/devel/lib/unit/fresh_double: unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libroscpp.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_signals-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_filesystem-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/librosconsole.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_regex-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/liblog4cxx.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libroscpp_serialization.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/librostime.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_date_time-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_system-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_thread-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/i386-linux-gnu/libpthread.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libxmlrpcpp.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libcpp_common.so
/home/rokon/renew/devel/lib/unit/fresh_double: /home/rokon/renew/devel/lib/libutils.so
/home/rokon/renew/devel/lib/unit/fresh_double: /home/rokon/renew/devel/lib/libmain.so
/home/rokon/renew/devel/lib/unit/fresh_double: /home/rokon/renew/devel/lib/libutils.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libroscpp.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_signals-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_filesystem-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/librosconsole.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_regex-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/liblog4cxx.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libroscpp_serialization.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/librostime.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_date_time-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_system-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/libboost_thread-mt.so
/home/rokon/renew/devel/lib/unit/fresh_double: /usr/lib/i386-linux-gnu/libpthread.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libxmlrpcpp.so
/home/rokon/renew/devel/lib/unit/fresh_double: /opt/ros/groovy/lib/libcpp_common.so
/home/rokon/renew/devel/lib/unit/fresh_double: unit/CMakeFiles/fresh_double.dir/build.make
/home/rokon/renew/devel/lib/unit/fresh_double: unit/CMakeFiles/fresh_double.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/rokon/renew/devel/lib/unit/fresh_double"
	cd /home/rokon/renew/build/unit && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fresh_double.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unit/CMakeFiles/fresh_double.dir/build: /home/rokon/renew/devel/lib/unit/fresh_double
.PHONY : unit/CMakeFiles/fresh_double.dir/build

unit/CMakeFiles/fresh_double.dir/requires: unit/CMakeFiles/fresh_double.dir/fresh_double.cpp.o.requires
.PHONY : unit/CMakeFiles/fresh_double.dir/requires

unit/CMakeFiles/fresh_double.dir/clean:
	cd /home/rokon/renew/build/unit && $(CMAKE_COMMAND) -P CMakeFiles/fresh_double.dir/cmake_clean.cmake
.PHONY : unit/CMakeFiles/fresh_double.dir/clean

unit/CMakeFiles/fresh_double.dir/depend:
	cd /home/rokon/renew/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rokon/renew/src /home/rokon/renew/src/unit /home/rokon/renew/build /home/rokon/renew/build/unit /home/rokon/renew/build/unit/CMakeFiles/fresh_double.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unit/CMakeFiles/fresh_double.dir/depend

