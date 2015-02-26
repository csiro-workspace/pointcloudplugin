SMOOTH SIGNED DISTANCE SURFACE RECONSTRUCTION SOFTWARE (VERSION 3.0)

CHANGES

Version 3.0 is a speed optimized revision of our old SSD software. In this version, 
we use a hash octree data structure based on Morton codes which is observed to allow much 
more efficient access to leaf cells and their neighbors. Daniel Moreno did significant 
amount of work to implement and integrate the hash octree into our code. In addition, 
he has made several changes in the code to optimize the running time and peak memory usage.

LICENSE

If you download the software we will assume that you have read and
agreed to the terms of the Licence Agreement

COMPILATION
We provide compiler independent configuration files, CMakeLists.txt. CMake is a cross-platform, 
open-source build system. If you don't have CMake software installed on your system, please do 
so by visiting cmake.org. Once you have CMake, follow these steps:

on UNIX:
1. Create a build directory 
	– mkdir build ; cd build
2. Configure the package for your system:
	– cmake -D CMAKE_BUILD_TYPE=RELEASE <location_of_the_source_directory>
3. Build the package:
	– make

on Windows (Visual Studio):
1. Create a build directory
2. Open the cmake gui, choose you platform (32/64 bit), then insert
	Where is the source code: <location_of_the_source_directory>
	Where to build binaries:  <location_of_build_directory_you_just_created>
3. Click configure 
4. Click generate
5. Open the project file ssd.sln and build ssd_recon in RELEASE

SAMPLE DATA SET

For testing purposes, we provide one oriented points data set "angel_points.ply".

USAGE

ssd_recon [options] [input file] [output file]

FOR EXAMPLE

ssd_recon -octreeLevels 9 -weights 1 1 1 angel_points.ply angel_mesh.ply

ARGUMENTS 

-octreeLevels [d]
	This positive integer is the maximum level of the octree that will be
	used for surface reconstruction. Running at level d corresponds to solving 
	on a voxel grid whose resolution is no larger than 2^d x 2^d x 2^d. 
	The default value is 8. 
-weights [w0 w1 w2]
	These positive floating point values specifies regularization
	parameters presented in the paper. We suggest using smaller lambda0
	than lambda1 and lambda2, particularly when data is noisy. Otherwise,
	lambda2 can be set a larger value than lambda0 and lambda1 produces a
	smoother result (optional). The default value is w0 = 1.0, w1 = 1.0, 
	and w2 = 1.0.
-solverTolerance [tol]
    This positive floating point value specifies the tolerance of the 
	iterative linear system solver. Values in the range [1.0e-7 - 1.0e-9] 
	can be used. The default value is 1.0e-8. 
-samplesPerNode [spn]
	This positive integer value specifies the minimum number of sample points 
	that should fall within an octree node. For noisy data, larger values in 
	the range [5.0 - 15.0] is needed to provide a smoother reconstruction. 
	Otherwise, small values in the range [1.0 - 5.0] can be used. 
	The default value is 1.0.
-debug 
	This is a flag to provide description of the reconstruction process	
  
[input file]
	This string is the name of the file from which the point set will be
	read stored in PLY file. We can parse this PLY file in either ASCII or
	binary format. The file should be a PLY file (either in ascii or
	binary format) with groups of 6, white space delimited, numbers (x-,
	y-, and z-coordinates of the point's position, followed by the x-, y-,
	and z-coordinates of the point's normal). The number of oriented point
	samples should be specified in the "element vertex" field. (Look at
	angel.ply provided in the samples folder)
[output file]
	This string is the name of the file to which the polygon mesh will be
	written. The file is written in PLY format.
	
ADDITIONAL ARGUMENTS

There are a few other optional command line arguments that we had been using 
for debugging purposes. Feel free to experiment.

VISUALIZATION

The input point cloud and our generated 3D models can be visualized
using Meshlab, a free 3D mesh processing software program. 
http://meshlab.sourceforge.net/
