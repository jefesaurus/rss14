# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/rss-student/RSS-I-group/lab5_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rss-student/RSS-I-group/lab5_msgs/build

# Utility rule file for ROSBUILD_genmsg_cpp.

CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/ColorMsg.h
CMakeFiles/ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h

../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: ../msg/GUIPointMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/ros/core/roslib/scripts/gendeps
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: ../msg/ColorMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: ../manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/ros/tools/rospack/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/ros/core/roslib/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h"
	/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py /home/rss-student/RSS-I-group/lab5_msgs/msg/GUIPointMsg.msg

../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: ../msg/GUILineMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/ros/core/roslib/scripts/gendeps
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: ../msg/ColorMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: ../manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/ros/tools/rospack/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/ros/core/roslib/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h"
	/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py /home/rss-student/RSS-I-group/lab5_msgs/msg/GUILineMsg.msg

../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: ../msg/GUIEraseMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/ros/core/roslib/scripts/gendeps
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg/String.msg
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: ../manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/ros/tools/rospack/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/ros/core/roslib/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h"
	/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py /home/rss-student/RSS-I-group/lab5_msgs/msg/GUIEraseMsg.msg

../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: ../msg/ColorMsg.msg
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/ros/core/roslib/scripts/gendeps
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: ../manifest.xml
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/ros/tools/rospack/manifest.xml
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/ros/core/roslib/manifest.xml
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
../msg_gen/cpp/include/lab5_msgs/ColorMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/lab5_msgs/ColorMsg.h"
	/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py /home/rss-student/RSS-I-group/lab5_msgs/msg/ColorMsg.msg

../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: ../msg/GUISegmentMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/ros/core/roslib/scripts/gendeps
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: ../msg/ColorMsg.msg
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: ../manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/ros/tools/rospack/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/ros/core/roslib/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/manifest.xml
../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h: /opt/ros/electric/stacks/ros_comm/messages/std_msgs/msg_gen/generated
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating ../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h"
	/opt/ros/electric/stacks/ros_comm/clients/cpp/roscpp/scripts/genmsg_cpp.py /home/rss-student/RSS-I-group/lab5_msgs/msg/GUISegmentMsg.msg

ROSBUILD_genmsg_cpp: CMakeFiles/ROSBUILD_genmsg_cpp
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUIPointMsg.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUILineMsg.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUIEraseMsg.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/ColorMsg.h
ROSBUILD_genmsg_cpp: ../msg_gen/cpp/include/lab5_msgs/GUISegmentMsg.h
ROSBUILD_genmsg_cpp: CMakeFiles/ROSBUILD_genmsg_cpp.dir/build.make
.PHONY : ROSBUILD_genmsg_cpp

# Rule to build all files generated by this target.
CMakeFiles/ROSBUILD_genmsg_cpp.dir/build: ROSBUILD_genmsg_cpp
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/build

CMakeFiles/ROSBUILD_genmsg_cpp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/clean

CMakeFiles/ROSBUILD_genmsg_cpp.dir/depend:
	cd /home/rss-student/RSS-I-group/lab5_msgs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rss-student/RSS-I-group/lab5_msgs /home/rss-student/RSS-I-group/lab5_msgs /home/rss-student/RSS-I-group/lab5_msgs/build /home/rss-student/RSS-I-group/lab5_msgs/build /home/rss-student/RSS-I-group/lab5_msgs/build/CMakeFiles/ROSBUILD_genmsg_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ROSBUILD_genmsg_cpp.dir/depend

