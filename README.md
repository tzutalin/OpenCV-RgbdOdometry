#Demo Opencv-RgbdOdometry

Requirement: OpenCV > 3.0

Download RGB	-D dataset from [TUM](http://vision.in.tum.de/data/datasets)

main.cpp will read the path of color and depth images from ./assoc.txt, then run OpenCV RgbdOdometry to compute visual odometry

The format of assoc.txt looks like:
```
timestamp1 rgb/[color_image_filename1] timestamp1 depth/[depth_image_filename1]
...
timestampN rgb/[color_image_filenameN] timestampN depth/[depth_image_filenameN]
```

You should change camera paramerts at the top of main.cpp
```
#define FOCUS_LENGTH 525.0
#define CX 319.5
#define CY 239.5
```
