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
CMAKE_SOURCE_DIR = /home/weepies/inf/Aim

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weepies/inf/Aim/build

# Include any dependencies generated for this target.
include libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/depend.make

# Include the progress variables for this target.
include libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/progress.make

# Include the compile flags for this target's objects.
include libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/flags.make

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/flags.make
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o: ../libHardWare/usbCapture/src/usb_capture_with_thread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weepies/inf/Aim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o -c /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_thread.cpp

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.i"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_thread.cpp > CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.i

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.s"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_thread.cpp -o CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.s

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.requires:

.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.requires

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.provides: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.requires
	$(MAKE) -f libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build.make libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.provides.build
.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.provides

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.provides.build: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o


libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/flags.make
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o: ../libHardWare/usbCapture/src/usb_capture_with_opencv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weepies/inf/Aim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o -c /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_opencv.cpp

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.i"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_opencv.cpp > CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.i

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.s"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture_with_opencv.cpp -o CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.s

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.requires:

.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.requires

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.provides: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.requires
	$(MAKE) -f libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build.make libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.provides.build
.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.provides

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.provides.build: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o


libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/flags.make
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o: ../libHardWare/usbCapture/src/usb_capture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weepies/inf/Aim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o -c /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture.cpp

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.i"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture.cpp > CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.i

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.s"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weepies/inf/Aim/libHardWare/usbCapture/src/usb_capture.cpp -o CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.s

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.requires:

.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.requires

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.provides: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.requires
	$(MAKE) -f libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build.make libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.provides.build
.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.provides

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.provides.build: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o


# Object files for target UsbCapture
UsbCapture_OBJECTS = \
"CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o" \
"CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o" \
"CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o"

# External object files for target UsbCapture
UsbCapture_EXTERNAL_OBJECTS =

libHardWare/usbCapture/libUsbCapture.a: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o
libHardWare/usbCapture/libUsbCapture.a: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o
libHardWare/usbCapture/libUsbCapture.a: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o
libHardWare/usbCapture/libUsbCapture.a: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build.make
libHardWare/usbCapture/libUsbCapture.a: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/weepies/inf/Aim/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libUsbCapture.a"
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && $(CMAKE_COMMAND) -P CMakeFiles/UsbCapture.dir/cmake_clean_target.cmake
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UsbCapture.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build: libHardWare/usbCapture/libUsbCapture.a

.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/build

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/requires: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_thread.cpp.o.requires
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/requires: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture_with_opencv.cpp.o.requires
libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/requires: libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/src/usb_capture.cpp.o.requires

.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/requires

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/clean:
	cd /home/weepies/inf/Aim/build/libHardWare/usbCapture && $(CMAKE_COMMAND) -P CMakeFiles/UsbCapture.dir/cmake_clean.cmake
.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/clean

libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/depend:
	cd /home/weepies/inf/Aim/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weepies/inf/Aim /home/weepies/inf/Aim/libHardWare/usbCapture /home/weepies/inf/Aim/build /home/weepies/inf/Aim/build/libHardWare/usbCapture /home/weepies/inf/Aim/build/libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : libHardWare/usbCapture/CMakeFiles/UsbCapture.dir/depend

