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
include camera/CMakeFiles/fix_double.dir/depend.make

# Include the progress variables for this target.
include camera/CMakeFiles/fix_double.dir/progress.make

# Include the compile flags for this target's objects.
include camera/CMakeFiles/fix_double.dir/flags.make

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o: camera/CMakeFiles/fix_double.dir/flags.make
camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o: /home/rokon/renew/src/camera/src/fix_double.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rokon/renew/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o"
	cd /home/rokon/renew/build/camera && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/fix_double.dir/src/fix_double.cpp.o -c /home/rokon/renew/src/camera/src/fix_double.cpp

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fix_double.dir/src/fix_double.cpp.i"
	cd /home/rokon/renew/build/camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rokon/renew/src/camera/src/fix_double.cpp > CMakeFiles/fix_double.dir/src/fix_double.cpp.i

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fix_double.dir/src/fix_double.cpp.s"
	cd /home/rokon/renew/build/camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rokon/renew/src/camera/src/fix_double.cpp -o CMakeFiles/fix_double.dir/src/fix_double.cpp.s

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.requires:
.PHONY : camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.requires

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.provides: camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.requires
	$(MAKE) -f camera/CMakeFiles/fix_double.dir/build.make camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.provides.build
.PHONY : camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.provides

camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.provides.build: camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o

# Object files for target fix_double
fix_double_OBJECTS = \
"CMakeFiles/fix_double.dir/src/fix_double.cpp.o"

# External object files for target fix_double
fix_double_EXTERNAL_OBJECTS =

/home/rokon/renew/devel/lib/camera/fix_double: camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libcv_bridge.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_videostab.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_video.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_superres.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_stitching.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_photo.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_objdetect.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_nonfree.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_ml.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_legacy.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_imgproc.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_highgui.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_gpu.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_flann.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_features2d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_core.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_contrib.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_calib3d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libimage_transport.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libmessage_filters.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libtinyxml.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libclass_loader.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libPocoFoundation.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/i386-linux-gnu/libdl.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libconsole_bridge.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libroscpp.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_signals-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_filesystem-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/librosconsole.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_regex-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/liblog4cxx.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libxmlrpcpp.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libroslib.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libroscpp_serialization.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/librostime.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_date_time-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_system-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/libboost_thread-mt.so
/home/rokon/renew/devel/lib/camera/fix_double: /usr/lib/i386-linux-gnu/libpthread.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libcpp_common.so
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_videostab.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_video.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_superres.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_stitching.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_photo.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_objdetect.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_nonfree.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_ml.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_legacy.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_imgproc.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_highgui.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_gpu.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_flann.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_features2d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_core.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_contrib.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_calib3d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_nonfree.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_gpu.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_photo.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_objdetect.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_legacy.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_video.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_ml.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_calib3d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_features2d.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_highgui.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_imgproc.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_flann.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: /opt/ros/groovy/lib/libopencv_core.so.2.4.9
/home/rokon/renew/devel/lib/camera/fix_double: camera/CMakeFiles/fix_double.dir/build.make
/home/rokon/renew/devel/lib/camera/fix_double: camera/CMakeFiles/fix_double.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/rokon/renew/devel/lib/camera/fix_double"
	cd /home/rokon/renew/build/camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fix_double.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
camera/CMakeFiles/fix_double.dir/build: /home/rokon/renew/devel/lib/camera/fix_double
.PHONY : camera/CMakeFiles/fix_double.dir/build

camera/CMakeFiles/fix_double.dir/requires: camera/CMakeFiles/fix_double.dir/src/fix_double.cpp.o.requires
.PHONY : camera/CMakeFiles/fix_double.dir/requires

camera/CMakeFiles/fix_double.dir/clean:
	cd /home/rokon/renew/build/camera && $(CMAKE_COMMAND) -P CMakeFiles/fix_double.dir/cmake_clean.cmake
.PHONY : camera/CMakeFiles/fix_double.dir/clean

camera/CMakeFiles/fix_double.dir/depend:
	cd /home/rokon/renew/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rokon/renew/src /home/rokon/renew/src/camera /home/rokon/renew/build /home/rokon/renew/build/camera /home/rokon/renew/build/camera/CMakeFiles/fix_double.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : camera/CMakeFiles/fix_double.dir/depend

