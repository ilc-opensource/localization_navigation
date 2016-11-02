# DemoSLN

DemSLN is the package of Visual SLAM code, created by Intel Labs China.

This package doesn't include Map generation code.

## Introduction

DemoSLN consists of:
- TreeGeneration: generate orb_db_yml.gz data file from given dataset.
- 2DMapGeneration: generate 2D map from given dataset 3D map.
- Localization: identify current location from given dataset.
- PathPlanning: given current position and target position, find a path connect the two.

## Build Introduction

Build system is CMake, make sure it's installed.

Both Windows and Linux are supported.

On Windows you can use create_x64/x86_msvc_prebuilt_3rdparty.bat to use prebuilt 3rdparty libraries and create Visual Studio 2013 solution file. Run it the VS file will be in build.

Or directly use CMake to build if you have necessary libraries installed on your system. There's an option to not use prebuilt libraries.

The libraries and executables will be in 'bin' folder under project root directory.

3rdparty libraries needed:
- [OpenCV](http://www.opencv.org)
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [g2o](https://github.com/RainerKuemmerle/g2o)
- [PCL](http://pointclouds.org/)
- [DBoW2](https://github.com/dorian3d/DBoW2)

Setup corresponding environment variables for them may be necessary according to your setup, please refer to CMake file.

## Prebuilt Windows libraries

For the ease of development on Windows, prebuilt Visual Studio 2013 3rdparty libraries or code are provided, versions of which are:
- boost           1.60
- VTK             5.10.1
- PCL             1.7.2
- OpenCV          3.1.0
- Eigen           3.2.8
- flann           1.8.4 (used for building PCL, not provided)
- g2o             c986d956a5860f9d162169166b72fdda6d2886d7
- librealsense    f019a4b4e36c426885ff8ae3eebf1ea79df60d05
- DBoW2 src       405f931c6e43d0ee83e493c7b64c8f653ecdec71

On Linux you can use CMake to build. For example in Linux:
```
mkdir build
cd build
cmake ..
make
```
There're options user can choose to build the project.

The libraries and executables will be in 'bin' folder under project root directory.

## Prebuilt Linux libraries
On ubuntu14.04, you can install them with:

- $sudo apt-get install libboost-dev libpcl-1.7-all-dev

And you may need to install opencv3.1.0 from (http://www.opencv.org), [librealsense](https://github.com/IntelRealSense/librealsense),[g2o](https://github.com/RainerKuemmerle/g2o), and compile these libraries.

## Sample Usage
In the following examples, the working directory is set to the project root directory and the file paths in configuration files are relative paths according to project root directory.

* 2DMapGeneration Usage:

  ```
  2DMapGeneration model camera
  e.g. 2DMapGeneration ./dataset/keyframe/g2o_linkbreak_.ply ./dataset/keyframe/cam.ply
  ```

* TreeGeneration Usage:

  ```
  TreeGeneration  RGB_images_director
  e.g. TreeGeneration ./dataset/keyframe/rgb
  ```
  
* Test_Localization Usage:

 Test_Localization is the demo program on Localization. 
 
   ```
  Test_Localization data_folder query_image_index
  e.g. Test_Localization ./dataset 11
  ```
  
 * Test_PathPlanning Usage:
 
 Test_PathPlanning is the demo program on path plan. 
 
   ```
  Test_PathPlanning map_image x1 y1 x2 y2 
  e.g. Test_Localization ./dataset/keyframe/map2d.png 100 200 300 400
  ```
  
## License

The code is released under 3-clause BSD License. Please see LICENSE file.


## Note

For limit of Github's file size, the debug 64-bit version of vtk libraries are not uploaded, please build these libraries by yourself.

Thanks.

