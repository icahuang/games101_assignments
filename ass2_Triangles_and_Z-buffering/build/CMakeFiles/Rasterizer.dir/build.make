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
CMAKE_SOURCE_DIR = /home/cs18/Desktop/assignment/ass2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cs18/Desktop/assignment/ass2/build

# Include any dependencies generated for this target.
include CMakeFiles/Rasterizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Rasterizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Rasterizer.dir/flags.make

CMakeFiles/Rasterizer.dir/main.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/assignment/ass2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Rasterizer.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rasterizer.dir/main.cpp.o -c /home/cs18/Desktop/assignment/ass2/main.cpp

CMakeFiles/Rasterizer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs18/Desktop/assignment/ass2/main.cpp > CMakeFiles/Rasterizer.dir/main.cpp.i

CMakeFiles/Rasterizer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs18/Desktop/assignment/ass2/main.cpp -o CMakeFiles/Rasterizer.dir/main.cpp.s

CMakeFiles/Rasterizer.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Rasterizer.dir/main.cpp.o.requires

CMakeFiles/Rasterizer.dir/main.cpp.o.provides: CMakeFiles/Rasterizer.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rasterizer.dir/build.make CMakeFiles/Rasterizer.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Rasterizer.dir/main.cpp.o.provides

CMakeFiles/Rasterizer.dir/main.cpp.o.provides.build: CMakeFiles/Rasterizer.dir/main.cpp.o


CMakeFiles/Rasterizer.dir/rasterizer.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/rasterizer.cpp.o: ../rasterizer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/assignment/ass2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Rasterizer.dir/rasterizer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rasterizer.dir/rasterizer.cpp.o -c /home/cs18/Desktop/assignment/ass2/rasterizer.cpp

CMakeFiles/Rasterizer.dir/rasterizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/rasterizer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs18/Desktop/assignment/ass2/rasterizer.cpp > CMakeFiles/Rasterizer.dir/rasterizer.cpp.i

CMakeFiles/Rasterizer.dir/rasterizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/rasterizer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs18/Desktop/assignment/ass2/rasterizer.cpp -o CMakeFiles/Rasterizer.dir/rasterizer.cpp.s

CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.requires:

.PHONY : CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.requires

CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.provides: CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rasterizer.dir/build.make CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.provides.build
.PHONY : CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.provides

CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.provides.build: CMakeFiles/Rasterizer.dir/rasterizer.cpp.o


CMakeFiles/Rasterizer.dir/Triangle.cpp.o: CMakeFiles/Rasterizer.dir/flags.make
CMakeFiles/Rasterizer.dir/Triangle.cpp.o: ../Triangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cs18/Desktop/assignment/ass2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Rasterizer.dir/Triangle.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Rasterizer.dir/Triangle.cpp.o -c /home/cs18/Desktop/assignment/ass2/Triangle.cpp

CMakeFiles/Rasterizer.dir/Triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Rasterizer.dir/Triangle.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cs18/Desktop/assignment/ass2/Triangle.cpp > CMakeFiles/Rasterizer.dir/Triangle.cpp.i

CMakeFiles/Rasterizer.dir/Triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Rasterizer.dir/Triangle.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cs18/Desktop/assignment/ass2/Triangle.cpp -o CMakeFiles/Rasterizer.dir/Triangle.cpp.s

CMakeFiles/Rasterizer.dir/Triangle.cpp.o.requires:

.PHONY : CMakeFiles/Rasterizer.dir/Triangle.cpp.o.requires

CMakeFiles/Rasterizer.dir/Triangle.cpp.o.provides: CMakeFiles/Rasterizer.dir/Triangle.cpp.o.requires
	$(MAKE) -f CMakeFiles/Rasterizer.dir/build.make CMakeFiles/Rasterizer.dir/Triangle.cpp.o.provides.build
.PHONY : CMakeFiles/Rasterizer.dir/Triangle.cpp.o.provides

CMakeFiles/Rasterizer.dir/Triangle.cpp.o.provides.build: CMakeFiles/Rasterizer.dir/Triangle.cpp.o


# Object files for target Rasterizer
Rasterizer_OBJECTS = \
"CMakeFiles/Rasterizer.dir/main.cpp.o" \
"CMakeFiles/Rasterizer.dir/rasterizer.cpp.o" \
"CMakeFiles/Rasterizer.dir/Triangle.cpp.o"

# External object files for target Rasterizer
Rasterizer_EXTERNAL_OBJECTS =

Rasterizer: CMakeFiles/Rasterizer.dir/main.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/rasterizer.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/Triangle.cpp.o
Rasterizer: CMakeFiles/Rasterizer.dir/build.make
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
Rasterizer: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
Rasterizer: CMakeFiles/Rasterizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cs18/Desktop/assignment/ass2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable Rasterizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Rasterizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Rasterizer.dir/build: Rasterizer

.PHONY : CMakeFiles/Rasterizer.dir/build

CMakeFiles/Rasterizer.dir/requires: CMakeFiles/Rasterizer.dir/main.cpp.o.requires
CMakeFiles/Rasterizer.dir/requires: CMakeFiles/Rasterizer.dir/rasterizer.cpp.o.requires
CMakeFiles/Rasterizer.dir/requires: CMakeFiles/Rasterizer.dir/Triangle.cpp.o.requires

.PHONY : CMakeFiles/Rasterizer.dir/requires

CMakeFiles/Rasterizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Rasterizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Rasterizer.dir/clean

CMakeFiles/Rasterizer.dir/depend:
	cd /home/cs18/Desktop/assignment/ass2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cs18/Desktop/assignment/ass2 /home/cs18/Desktop/assignment/ass2 /home/cs18/Desktop/assignment/ass2/build /home/cs18/Desktop/assignment/ass2/build /home/cs18/Desktop/assignment/ass2/build/CMakeFiles/Rasterizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Rasterizer.dir/depend

