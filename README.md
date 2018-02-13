CMPUT 511: Project - Point Cloud Normal Vector Estimation
**Bernard Llanos**

**December 8, 2017**
--------------------------

## Description
- A MATLAB script for generating point clouds, where the points are arranged
  along curves, describing superellipsoids
- An OpenGL application which reads curve-structured point cloud files incrementally,
  and estimates normal vectors at point positions, using several different
  algorithms
  - Ignoring normal vector estimation capabilities, this program can be used
    to view point clouds produced by the 3D probing project
    (https://github.com/bllanos/linear-probe).
- A MATLAB script for quantitative evaluation of normal vector estimation
  on the curve-structured point clouds

## Dependencies
- This program will not run in the CSC 1-59 lab environment, because it requires
  additional libraries.
- MATLAB is required to run the data generation and evaluation components of the
  project. MATLAB 2017b was used for development, but a much older version of
  MATLAB should be sufficient. No additional MathWorks toolboxes should be required.
- Development was carried out on Linux (Ubuntu 16.04, for instance).

### Libraries used by the OpenGL application
- Computational Geometry Algorithms Library (Ubuntu packages 'libcgal*'),
  version 4.11-1 (https://www.cgal.org/)
  - Depends on the GMP and Boost libraries (https://doc.cgal.org/latest/Manual/installation.html)
- Eigen3 (Ubuntu package 'libeigen3-dev'), version 3.3
  (http://eigen.tuxfamily.org/index.php?title=Main_Page)
- FreeGLUT (Ubuntu packages 'freeglut3*'), version 2.8.1
  (http://freeglut.sourceforge.net/)
- OpenGL 2.1 or higher
- OpenGL Extension Wrangler Library (Ubuntu packages 'libglew*'), version 1.13
  (http://glew.sourceforge.net/)

## Building
- Run `make pointCloudViewer` to produce the executable 'pointCloudViewer'.

## Usage
- Run './MATLAB/SampleSuperellipsoids.m' to generate point cloud data files,
  which presently will be saved in './superellipsoids/'.
- `./pointCloudViewer $FILENAME`, where 'FILENAME' is the path to one of the
  point cloud data files. A graphical application will launch and start
  building the point cloud on screen.
  - Press 'g' and 'G' to cycle back and forth between normal estimation algorithms.
    Some algorithms are only implemented in batch form, and will update all points
    when they are activated. As the point cloud continues to be built, these
    batch algorithms are not run except when they are re-selected using the
    'g' and 'G' keys.
  - Press 'e' or 'E' to save the estimated normal vectors at points in the point
    cloud to the file './results.csv'. Results are saved for all normal vector
    estimation algorithms at once.
  - A full list of keyboard controls is provided below.
- Run './MATLAB/Evaluation.m' to summarize the errors of the normal vector
  estimation algorithms. The script presently performs its calculations on the
  data in './results.csv'.

## './pointCloudViewer' display
- The curve of points which is currently being read is displayed in cyan.
- Normal vectors are implicitly represented by lighting the scene. A single point
  light source is to the right of, and above, the camera.
- Existing curves are displayed in green to yellow if their normal vectors deviate
  from the reference normal vectors by zero to 10 degrees, respectively.
- Existing curves are displayed in yellow to red if their normal vectors deviate
  from the reference normal vectors by 10 to 45 (or greater) degrees, respectively.

## './pointCloudViewer' input file format
A CSV file with the following columns:
- Curve index: The actual value is ignored, but a change in values between lines
  results in a new curves being created
- Point index along the curve (ignored)
- x-coordinate of the point position
- y-coordinate of the point position
- z-coordinate of the point position
- x-component of the point's normal vector hypothesis
- y-component of the point's normal vector hypothesis
- z-component of the point's normal vector hypothesis

The normal vector hypothesis is a unit vector within 90 degrees of the true
normal vector.

## './pointCloudViewer' output file format
Identical to the input file format, but with the following three additional
columns:
- x-component of the curve tangent vector at the point
- y-component of the curve tangent vector at the point
- z-component of the curve tangent vector at the point

The normal vectors in the output files represent estimated normal vectors,
as opposed to normal vector hypotheses.

## Keyboard Controls

### Letter keys
- 'a'/'A': Rotate the camera around its local Y-axis, in either direction,
  respectively
- 'b': Decrease width of rasterized lines
- 'B': Increase width of rasterized lines
- 'c'/'C': Translate the camera along its -Y and +Y directions, respectively
- 'd'/'D': Translate the camera along its -X and +X directions, respectively
- 'e'/'E': Write normal vectors, point positions, and estimated point tangent
  vectors to a file for use in quantitative evaluation.
- 'f': Turn off fog effect
- 'F': Turn on fog effect (default)
- 'g'/'G': Cycle back and forth between the different normal estimation algorithms.
  There are 6 algorithms available, including simply using the reference
  normal vectors.
- 'l'/'L': Rotate the camera around its local Z-axis, in either direction,
  respectively
- 'n'/'N': Translate the point cloud along its -Z and +Z directions,
  respectively
- 'p': Resume building the point cloud from the input file
- 'P': Pause building the point cloud from the input file
- 'q': Quit
- 'r'/'R': Rotate the point cloud around its local Z-axis, in either direction,
  respectively
- 's': Decrease size of rasterized points
- 'S': Increase size of rasterized points
- 't'/'T': Rotate the camera around its local X-axis, in either direction,
  respectively
- 'u'/'U': Rotate the point cloud around its local X-axis, in either direction,
  respectively
- 'v': Switch to orthographic projection
- 'V': Switch to the default perspective projection
- 'w'/'W': Write the point cloud data out for debugging purposes. The output
  file, './out.csv', is the same as the input file, except that the lines starting
  with 'vn' and '#t' are the estimated surface normal vectors and
  curve tangent vectors, respectively.
- 'x': Reset camera pose, clear the point cloud, and pause. When the program
  is resumed, the point cloud file will be read again starting from
  the beginning.
- 'y'/'Y': Rotate the point cloud around its local Y-axis, in either direction,
  respectively
- 'z'/'Z': Translate the camera along its -Z and +Z directions, respectively

### Non-letter keys
- '-': Decrease the rate at which the point cloud is built
- '+': Increase the rate at which the point cloud is built
- 'DOWN'/'UP': Translate the point cloud along its -Y and +Y directions,
  respectively
- 'LEFT'/'RIGHT': Translate the point cloud along its -X and +X directions,
  respectively

## References
Each header file lists references consulted when creating it and
the corresponding '.cpp' file (if there is one).

I developed the project code and my code for Assignments 1-3 together.
