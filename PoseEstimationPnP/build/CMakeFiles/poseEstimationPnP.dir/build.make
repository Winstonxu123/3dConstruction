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
CMAKE_SOURCE_DIR = /home/xu/3weiConstruction/PoseEstimationPnP

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xu/3weiConstruction/PoseEstimationPnP/build

# Include any dependencies generated for this target.
include CMakeFiles/poseEstimationPnP.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/poseEstimationPnP.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/poseEstimationPnP.dir/flags.make

CMakeFiles/poseEstimationPnP.dir/main.cpp.o: CMakeFiles/poseEstimationPnP.dir/flags.make
CMakeFiles/poseEstimationPnP.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xu/3weiConstruction/PoseEstimationPnP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/poseEstimationPnP.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/poseEstimationPnP.dir/main.cpp.o -c /home/xu/3weiConstruction/PoseEstimationPnP/main.cpp

CMakeFiles/poseEstimationPnP.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/poseEstimationPnP.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xu/3weiConstruction/PoseEstimationPnP/main.cpp > CMakeFiles/poseEstimationPnP.dir/main.cpp.i

CMakeFiles/poseEstimationPnP.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/poseEstimationPnP.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xu/3weiConstruction/PoseEstimationPnP/main.cpp -o CMakeFiles/poseEstimationPnP.dir/main.cpp.s

CMakeFiles/poseEstimationPnP.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/poseEstimationPnP.dir/main.cpp.o.requires

CMakeFiles/poseEstimationPnP.dir/main.cpp.o.provides: CMakeFiles/poseEstimationPnP.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/poseEstimationPnP.dir/build.make CMakeFiles/poseEstimationPnP.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/poseEstimationPnP.dir/main.cpp.o.provides

CMakeFiles/poseEstimationPnP.dir/main.cpp.o.provides.build: CMakeFiles/poseEstimationPnP.dir/main.cpp.o


# Object files for target poseEstimationPnP
poseEstimationPnP_OBJECTS = \
"CMakeFiles/poseEstimationPnP.dir/main.cpp.o"

# External object files for target poseEstimationPnP
poseEstimationPnP_EXTERNAL_OBJECTS =

poseEstimationPnP: CMakeFiles/poseEstimationPnP.dir/main.cpp.o
poseEstimationPnP: CMakeFiles/poseEstimationPnP.dir/build.make
poseEstimationPnP: libpart.so
poseEstimationPnP: /usr/local/lib/libopencv_dnn.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_ml.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_objdetect.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_shape.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_stitching.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_superres.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_videostab.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_viz.so.3.3.0
poseEstimationPnP: /usr/lib/x86_64-linux-gnu/libcxsparse.so
poseEstimationPnP: /usr/local/lib/libopencv_calib3d.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_features2d.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_flann.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_highgui.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_photo.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_video.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_videoio.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_imgcodecs.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_imgproc.so.3.3.0
poseEstimationPnP: /usr/local/lib/libopencv_core.so.3.3.0
poseEstimationPnP: CMakeFiles/poseEstimationPnP.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xu/3weiConstruction/PoseEstimationPnP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable poseEstimationPnP"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/poseEstimationPnP.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/poseEstimationPnP.dir/build: poseEstimationPnP

.PHONY : CMakeFiles/poseEstimationPnP.dir/build

CMakeFiles/poseEstimationPnP.dir/requires: CMakeFiles/poseEstimationPnP.dir/main.cpp.o.requires

.PHONY : CMakeFiles/poseEstimationPnP.dir/requires

CMakeFiles/poseEstimationPnP.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/poseEstimationPnP.dir/cmake_clean.cmake
.PHONY : CMakeFiles/poseEstimationPnP.dir/clean

CMakeFiles/poseEstimationPnP.dir/depend:
	cd /home/xu/3weiConstruction/PoseEstimationPnP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xu/3weiConstruction/PoseEstimationPnP /home/xu/3weiConstruction/PoseEstimationPnP /home/xu/3weiConstruction/PoseEstimationPnP/build /home/xu/3weiConstruction/PoseEstimationPnP/build /home/xu/3weiConstruction/PoseEstimationPnP/build/CMakeFiles/poseEstimationPnP.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/poseEstimationPnP.dir/depend

