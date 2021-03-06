# SDF-2-SDF
Implementation of the paper " SDF-2-SDF: Highly Accurate 3D Object Reconstruction".

## Building Instructions
### Dependencies 
+ OpenCV 3.+ (used for depth image IO)
+ Eigen (used for matrices operations)
+ Sophus (used for Lie algebras)
+ VTK 9.+ (used for visualization)
+ c++11 or above (used for std::thread)

### Build
```mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make```
Building **RELEASE** version is strongly recommended for fast execution.
### Run
Firstly you need to download dataset from [here](http://campar.in.tum.de/personal/slavcheva/3d-printed-dataset/index.html). Put it under the same level as *src* and *include*. Also/Or you need to specify the directory path in *algorithms.cpp*.

Then run ```./sdf-2-sdf```

## Interaction
All the key/mouse interactions in VTK windows are specified at VTK's website. Basically, press *t* to enable trackball style, use the mouse wheel to zoom in/out, press the mouse wheel(mouse mid button) to drage the scene.
