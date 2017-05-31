# RGBD-Odometry (Visual Odometry based RGB-D images)

### Requirement
[OpenCV >= 3.0](http://tzutalin.blogspot.tw/2016/01/installing-opencv-310-and-contrib-lib.html)

### Setup
Download RGB-D dataset from [TUM](http://vision.in.tum.de/data/datasets) or [KTTI](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

 [main.cpp](https://github.com/tzutalin/OpenCV-RgbdOdometry/blob/master/src/main.cpp#L160) will read the path of color and depth images from ./assoc.txt, then run OpenCV RgbdOdometry to compute visual odometry

The format of assoc.txt looks like:
```
timestamp1 rgb/[color_image_filename1] timestamp1 depth/[depth_image_filename1]
...
timestampN rgb/[color_image_filenameN] timestampN depth/[depth_image_filenameN]
```

You should change camera paramerts at the top of [main.cpp](https://github.com/tzutalin/OpenCV-RgbdOdometry/blob/master/src/main.cpp#L24)
```
#define FOCUS_LENGTH 525.0
#define CX 319.5
#define CY 239.5
```

### Build & Run
For the first time, you should download the dataset. You can use the below command

`$ python tools/download_dataset.py`

Start building

`$ mkdir -p build; cd build`

`$ cmake ..; make`

Start running

`$ cd [Opencv-RgbdOdometry]`

Create assoc.txt having synchronized rgb and depth images

`$ cd rgbd_dataset_freiburg2_pioneer_slam3`

`$ python associate.py rgb.txt depth.txt > assoc.txt`

`$ ../build/rgbd-odometry`

### Demo video
[![Demo video](https://j.gifs.com/0RDJgK.gif)](https://www.youtube.com/watch?v=NS2L7_uHTAo&feature=youtu.be)

[![Demo video](https://j.gifs.com/lYEqx5.gif)](https://www.youtube.com/watch?v=NS2L7_uHTAo&feature=youtu.be)
