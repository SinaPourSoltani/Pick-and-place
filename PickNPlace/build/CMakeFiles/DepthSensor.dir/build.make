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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.19.0_1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.19.0_1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build

# Include any dependencies generated for this target.
include CMakeFiles/DepthSensor.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DepthSensor.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DepthSensor.dir/flags.make

CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o: CMakeFiles/DepthSensor.dir/flags.make
CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o: ../pickAndPlace.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o -c /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/pickAndPlace.cpp

CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/pickAndPlace.cpp > CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.i

CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/pickAndPlace.cpp -o CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.s

# Object files for target DepthSensor
DepthSensor_OBJECTS = \
"CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o"

# External object files for target DepthSensor
DepthSensor_EXTERNAL_OBJECTS =

DepthSensor: CMakeFiles/DepthSensor.dir/pickAndPlace.cpp.o
DepthSensor: CMakeFiles/DepthSensor.dir/build.make
DepthSensor: /usr/local/lib/libpcl_apps.dylib
DepthSensor: /usr/local/lib/libpcl_outofcore.dylib
DepthSensor: /usr/local/lib/libpcl_people.dylib
DepthSensor: /usr/local/lib/libboost_system-mt.dylib
DepthSensor: /usr/local/lib/libboost_filesystem-mt.dylib
DepthSensor: /usr/local/lib/libboost_date_time-mt.dylib
DepthSensor: /usr/local/lib/libboost_iostreams-mt.dylib
DepthSensor: /usr/local/lib/libboost_regex-mt.dylib
DepthSensor: /usr/local/lib/libqhull_p.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkChartsCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInfovisCore-8.2.1.dylib
DepthSensor: /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk/usr/lib/libz.tbd
DepthSensor: /usr/local/lib/libjpeg.dylib
DepthSensor: /usr/local/lib/libpng.dylib
DepthSensor: /usr/local/lib/libtiff.dylib
DepthSensor: /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk/usr/lib/libexpat.tbd
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOGeometry-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOLegacy-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOPLY-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingLOD-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkViewsContext2D-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkViewsCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingContextOpenGL2-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingOpenGL2-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkglew-8.2.1.dylib
DepthSensor: /usr/local/lib/libflann_cpp.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libyaobi.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libpqp.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_qhull.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_csgjs.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_algorithms.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_pathplanners.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_pathoptimization.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_simulation.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_opengl.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_assembly.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_task.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_calibration.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_csg.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_control.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_proximitystrategies.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_core.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_common.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_math.dylib
DepthSensor: /usr/local/lib/liblua5.3.dylib
DepthSensor: /usr/local/lib/libassimp.dylib
DepthSensor: /usr/local/lib/libxerces-c.dylib
DepthSensor: /usr/local/lib/libboost_filesystem-mt.dylib
DepthSensor: /usr/local/lib/libboost_regex-mt.dylib
DepthSensor: /usr/local/lib/libboost_serialization-mt.dylib
DepthSensor: /usr/local/lib/libboost_system-mt.dylib
DepthSensor: /usr/local/lib/libboost_thread-mt.dylib
DepthSensor: /usr/local/lib/libboost_program_options-mt.dylib
DepthSensor: /usr/local/lib/libboost_chrono-mt.dylib
DepthSensor: /usr/local/lib/libboost_date_time-mt.dylib
DepthSensor: /usr/local/lib/libboost_atomic-mt.dylib
DepthSensor: /usr/local/lib/libboost_test_exec_monitor-mt.a
DepthSensor: /usr/local/lib/libboost_unit_test_framework-mt.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws_robworkstudioapp.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws_workcelleditor.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libqtpropertybrowser.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libyaobi.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libpqp.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_qhull.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_csgjs.a
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_algorithms.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_pathplanners.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_pathoptimization.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_simulation.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_opengl.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_assembly.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_task.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_calibration.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_csg.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_control.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_proximitystrategies.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_core.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_common.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWork/libs/relwithdebinfo/libsdurw_math.dylib
DepthSensor: /usr/local/lib/liblua5.3.dylib
DepthSensor: /usr/local/lib/libassimp.dylib
DepthSensor: /usr/local/lib/libxerces-c.dylib
DepthSensor: /usr/local/lib/libboost_filesystem-mt.dylib
DepthSensor: /usr/local/lib/libboost_regex-mt.dylib
DepthSensor: /usr/local/lib/libboost_serialization-mt.dylib
DepthSensor: /usr/local/lib/libboost_system-mt.dylib
DepthSensor: /usr/local/lib/libboost_thread-mt.dylib
DepthSensor: /usr/local/lib/libboost_program_options-mt.dylib
DepthSensor: /usr/local/lib/libboost_chrono-mt.dylib
DepthSensor: /usr/local/lib/libboost_date_time-mt.dylib
DepthSensor: /usr/local/lib/libboost_atomic-mt.dylib
DepthSensor: /usr/local/lib/libboost_test_exec_monitor-mt.a
DepthSensor: /usr/local/lib/libboost_unit_test_framework-mt.dylib
DepthSensor: /usr/local/opt/qt5/lib/QtOpenGL.framework/QtOpenGL
DepthSensor: /usr/local/Frameworks/Python.framework/Versions/3.9/lib/libpython3.9.dylib
DepthSensor: /usr/local/lib/libpcl_surface.dylib
DepthSensor: /usr/local/lib/libpcl_keypoints.dylib
DepthSensor: /usr/local/lib/libpcl_tracking.dylib
DepthSensor: /usr/local/lib/libpcl_recognition.dylib
DepthSensor: /usr/local/lib/libpcl_registration.dylib
DepthSensor: /usr/local/lib/libpcl_stereo.dylib
DepthSensor: /usr/local/lib/libpcl_segmentation.dylib
DepthSensor: /usr/local/lib/libpcl_features.dylib
DepthSensor: /usr/local/lib/libpcl_filters.dylib
DepthSensor: /usr/local/lib/libpcl_sample_consensus.dylib
DepthSensor: /usr/local/lib/libpcl_ml.dylib
DepthSensor: /usr/local/lib/libpcl_visualization.dylib
DepthSensor: /usr/local/lib/libpcl_search.dylib
DepthSensor: /usr/local/lib/libpcl_kdtree.dylib
DepthSensor: /usr/local/lib/libpcl_io.dylib
DepthSensor: /usr/local/lib/libpcl_octree.dylib
DepthSensor: /usr/local/lib/libpcl_common.dylib
DepthSensor: /usr/local/lib/libboost_iostreams-mt.dylib
DepthSensor: /usr/local/lib/libqhull_p.dylib
DepthSensor: /usr/local/lib/libjpeg.dylib
DepthSensor: /usr/local/lib/libpng.dylib
DepthSensor: /usr/local/lib/libtiff.dylib
DepthSensor: /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk/usr/lib/libexpat.tbd
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws_robworkstudioapp.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws_workcelleditor.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libsdurws.dylib
DepthSensor: /Users/sinapoursoltani/RobWork/RobWorkStudio/libs/relwithdebinfo/libqtpropertybrowser.a
DepthSensor: /usr/local/Frameworks/Python.framework/Versions/3.9/lib/libpython3.9.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInteractionWidgets-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersModeling-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkInteractionStyle-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersExtraction-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersStatistics-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingFourier-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersHybrid-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingGeneral-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingSources-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingHybrid-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOImage-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkDICOMParser-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkmetaio-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingAnnotation-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingColor-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingVolume-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOXML-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOXMLParser-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkIOCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkdoubleconversion-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtklz4-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtklzma-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkImagingCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingContext2D-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingFreeType-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkfreetype-8.2.1.dylib
DepthSensor: /Library/Developer/CommandLineTools/SDKs/MacOSX10.15.sdk/usr/lib/libz.tbd
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkRenderingCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonColor-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersGeometry-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersSources-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersGeneral-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonComputationalGeometry-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkFiltersCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonExecutionModel-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonDataModel-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonTransforms-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonMisc-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonMath-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonSystem-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtkCommonCore-8.2.1.dylib
DepthSensor: /usr/local/Cellar/vtk@8.2/8.2.0_1/lib/libvtksys-8.2.1.dylib
DepthSensor: /usr/local/opt/qt5/lib/QtWidgets.framework/QtWidgets
DepthSensor: /usr/local/opt/qt5/lib/QtGui.framework/QtGui
DepthSensor: /usr/local/opt/qt5/lib/QtCore.framework/QtCore
DepthSensor: CMakeFiles/DepthSensor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable DepthSensor"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DepthSensor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DepthSensor.dir/build: DepthSensor

.PHONY : CMakeFiles/DepthSensor.dir/build

CMakeFiles/DepthSensor.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DepthSensor.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DepthSensor.dir/clean

CMakeFiles/DepthSensor.dir/depend:
	cd /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build /Users/sinapoursoltani/Documents/SyddanskUniversitet/Kandidat/01_Semester/RoVi/Project/Pick-and-place/PickNPlace/build/CMakeFiles/DepthSensor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DepthSensor.dir/depend
