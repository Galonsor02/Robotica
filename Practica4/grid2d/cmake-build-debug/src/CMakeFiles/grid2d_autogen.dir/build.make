# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /snap/clion/305/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/305/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/usuario/robocomp/components/Grupo3/Practica4/grid2d

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug

# Utility rule file for grid2d_autogen.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/grid2d_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/grid2d_autogen.dir/progress.make

src/CMakeFiles/grid2d_autogen: src/grid2d_autogen/timestamp

src/grid2d_autogen/timestamp: /usr/lib/qt6/libexec/moc
src/grid2d_autogen/timestamp: /usr/lib/qt6/libexec/uic
src/grid2d_autogen/timestamp: src/CMakeFiles/grid2d_autogen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target grid2d"
	cd /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src && /snap/clion/305/bin/cmake/linux/x64/bin/cmake -E cmake_autogen /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src/CMakeFiles/grid2d_autogen.dir/AutogenInfo.json Debug
	cd /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src && /snap/clion/305/bin/cmake/linux/x64/bin/cmake -E touch /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src/grid2d_autogen/timestamp

grid2d_autogen: src/CMakeFiles/grid2d_autogen
grid2d_autogen: src/grid2d_autogen/timestamp
grid2d_autogen: src/CMakeFiles/grid2d_autogen.dir/build.make
.PHONY : grid2d_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/grid2d_autogen.dir/build: grid2d_autogen
.PHONY : src/CMakeFiles/grid2d_autogen.dir/build

src/CMakeFiles/grid2d_autogen.dir/clean:
	cd /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/grid2d_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/grid2d_autogen.dir/clean

src/CMakeFiles/grid2d_autogen.dir/depend:
	cd /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/usuario/robocomp/components/Grupo3/Practica4/grid2d /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/src /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src /home/usuario/robocomp/components/Grupo3/Practica4/grid2d/cmake-build-debug/src/CMakeFiles/grid2d_autogen.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/CMakeFiles/grid2d_autogen.dir/depend

