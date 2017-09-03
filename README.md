[![Build Status](https://travis-ci.com/kaszczesny/robustified-rototranslation.svg?token=LDEBpPqPEdFYWy3sTZpg&branch=master)](https://travis-ci.com/kaszczesny/robustified-rototranslation)

# robustified-rototranslation
Master of Science Thesis by Krzysztof Szczęsny and Jan Twardowski: *Algorithm for visual odometry*, 2017.
AGH University of Science and Technology, Department of Telecommunications.
Supervisor: Jarosław Bułat, PhD.

The algorithm is based on code and paper by J.J. Tarrio & S. Pedre.

# Installation
Run `./install.sh` (without sudo - because we don't want the build files to be owned by root). It will:
 * install dependencies and tools,
 * fetch OpenCV 3.1 sources,
 * compile OpenCV (for the moment without `contrib`),
 * install OpenCV,
 * generate MEX files for Octave (generated `octave/setup_opencv.m` must be run every time to include them in path).

# Subprojects
Each subproject contains `run.sh` script to build it. They are all checked by Travis.
 * `octave/` - the algorithm prototype. Though not included in CI, could be [in](https://github.com/scottclowe/MOxUnit) [future](https://github.com/scottclowe/matlab-continuous-integration),
 * ~~`android/` - an Android project that uses our library,~~ (obsolete)
 * ~~`library/` - the C++ library,~~ (obsolete)
 * `tex/` - thesis sources.

# Other folders
 * `data/` - ~~unit test~~ bulk data too large for normal storage (whole directory uses Git LFS), yet too small for external download,
 * `opencv/` - OpenCV 3.1 sources & binaries (after installation binaries are also available in `/usr/local`). 

# Running the Octave prototype
 * adjust the `Config.m` file,
 * run `main.m` script.

# Building the C++ library (obsolete)
In the `library/` directory execute:
 * `make all` to compile the library,
 * `make test` to compile the library and unit tests (`./test` executable),
 * `make clean` to remove compiled files,
 * `./run.sh` to compile the library and unit tests & run them (this will run special UT version for Travis - with no windows),
 * `make doc` to generate documentation.

# Remarks
OpenCV 3.1 was chosen due to [mexopencv](https://github.com/kyamagu/mexopencv) constraints.

