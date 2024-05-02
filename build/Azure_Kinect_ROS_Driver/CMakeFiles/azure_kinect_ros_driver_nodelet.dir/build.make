# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xingtian/catkin_kin/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xingtian/catkin_kin/build

# Include any dependencies generated for this target.
include Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/compiler_depend.make

# Include the progress variables for this target.
include Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/progress.make

# Include the compile flags for this target's objects.
include Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/flags.make

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/flags.make
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o: /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_bridge_nodelet.cpp
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingtian/catkin_kin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o -MF CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o.d -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o -c /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_bridge_nodelet.cpp

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.i"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_bridge_nodelet.cpp > CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.i

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.s"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_bridge_nodelet.cpp -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.s

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/flags.make
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o: /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingtian/catkin_kin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o -MF CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o.d -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o -c /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.i"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp > CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.i

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.s"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.s

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/flags.make
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o: /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device_params.cpp
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingtian/catkin_kin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o -MF CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o.d -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o -c /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device_params.cpp

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.i"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device_params.cpp > CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.i

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.s"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device_params.cpp -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.s

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/flags.make
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o: /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_calibration_transform_data.cpp
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xingtian/catkin_kin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o -MF CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o.d -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o -c /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_calibration_transform_data.cpp

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.i"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_calibration_transform_data.cpp > CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.i

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.s"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver/src/k4a_calibration_transform_data.cpp -o CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.s

# Object files for target azure_kinect_ros_driver_nodelet
azure_kinect_ros_driver_nodelet_OBJECTS = \
"CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o" \
"CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o" \
"CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o" \
"CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o"

# External object files for target azure_kinect_ros_driver_nodelet
azure_kinect_ros_driver_nodelet_EXTERNAL_OBJECTS =

/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_bridge_nodelet.cpp.o
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device.cpp.o
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_ros_device_params.cpp.o
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/src/k4a_calibration_transform_data.cpp.o
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/build.make
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libk4arecord.so.1.4.1
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libimage_transport.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libimage_geometry.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_face.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_hfs.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_quality.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_text.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_tracking.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/liborocos-kdl.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/liborocos-kdl.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libtf2_ros.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libactionlib.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libmessage_filters.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libtf2.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libnodeletlib.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libbondcpp.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libuuid.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libclass_loader.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libPocoFoundation.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libdl.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libroslib.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/librospack.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libpython3.8.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /home/xingtian/catkin_kin/devel/lib/libcv_bridge.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libcamera_info_manager.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libcamera_calibration_parsers.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libroscpp.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/librosconsole.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/librostime.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /opt/ros/noetic/lib/libcpp_common.so
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libk4a.so.1.4.1
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_gapi.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_video.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_dnn.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: /usr/lib/aarch64-linux-gnu/libopencv_core.so.4.5.4
/home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so: Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xingtian/catkin_kin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library /home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so"
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/azure_kinect_ros_driver_nodelet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/build: /home/xingtian/catkin_kin/devel/lib/libazure_kinect_ros_driver_nodelet.so
.PHONY : Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/build

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/clean:
	cd /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver && $(CMAKE_COMMAND) -P CMakeFiles/azure_kinect_ros_driver_nodelet.dir/cmake_clean.cmake
.PHONY : Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/clean

Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/depend:
	cd /home/xingtian/catkin_kin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xingtian/catkin_kin/src /home/xingtian/catkin_kin/src/Azure_Kinect_ROS_Driver /home/xingtian/catkin_kin/build /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver /home/xingtian/catkin_kin/build/Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Azure_Kinect_ROS_Driver/CMakeFiles/azure_kinect_ros_driver_nodelet.dir/depend
