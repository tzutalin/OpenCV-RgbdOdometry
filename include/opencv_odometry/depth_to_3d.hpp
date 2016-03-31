/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.
                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)
Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.
This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

#ifndef DEPTH_TO_3D_HPP_
#define DEPTH_TO_3D_HPP_
#include <opencv2/core.hpp>
#include <limits.h>
#include <opencv_odometry/rgbd.hpp>
namespace cv
{
namespace rgbd
{
using namespace cv;

/** If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
 * by 1000 to get a depth in meters, and the values 0 are converted to std::numeric_limits<float>::quiet_NaN()
 * Otherwise, the image is simply converted to floats
 * @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
 *              (as done with the Microsoft Kinect), it is assumed in meters)
 * @param the desired output depth (floats or double)
 * @param out The rescaled float depth image
 */
template<typename T>
void
rescaleDepthTemplated(const Mat& in, Mat& out);

template<>
inline void
rescaleDepthTemplated<float>(const Mat& in, Mat& out)
{
  rescaleDepth(in, CV_32F, out);
}

template<>
inline void
rescaleDepthTemplated<double>(const Mat& in, Mat& out)
{
  rescaleDepth(in, CV_64F, out);
}

/**
 * @param depth the depth image, containing depth with the value T
 * @param the mask, containing CV_8UC1
 */
template <typename T>
size_t
convertDepthToFloat(const cv::Mat& depth, const cv::Mat& mask, float scale, cv::Mat_<float> &u_mat, cv::Mat_<float> &v_mat, cv::Mat_<float> &z_mat)
{
  CV_Assert (depth.size == mask.size);

  cv::Size depth_size = depth.size();

  cv::Mat_<uchar> uchar_mask = mask;

  if (mask.depth() != CV_8U)
    mask.convertTo(uchar_mask, CV_8U);

  u_mat = cv::Mat_<float>(depth_size.area(), 1);
  v_mat = cv::Mat_<float>(depth_size.area(), 1);
  z_mat = cv::Mat_<float>(depth_size.area(), 1);

  // Raw data from the Kinect has int
  size_t n_points = 0;

  for (int v = 0; v < depth_size.height; v++)
  {
    uchar* r = uchar_mask.ptr<uchar>(v, 0);

    for (int u = 0; u < depth_size.width; u++, ++r)
      if (*r)
      {
        u_mat((int)n_points, 0) = (float)u;
        v_mat((int)n_points, 0) = (float)v;
        T depth_i = depth.at<T>(v, u);

        if (cvIsNaN((float)depth_i) || (depth_i == std::numeric_limits<T>::min()) || (depth_i == std::numeric_limits<T>::max()))
          z_mat((int)n_points, 0) = std::numeric_limits<float>::quiet_NaN();
        else
          z_mat((int)n_points, 0) = depth_i * scale;

        ++n_points;
      }
  }

  return n_points;
}

/**
 * @param depth the depth image, containing depth with the value T
 * @param the mask, containing CV_8UC1
 */
template <typename T>
void
convertDepthToFloat(const cv::Mat& depth, float scale, const cv::Mat &uv_mat, cv::Mat_<float> &z_mat)
{
  z_mat = cv::Mat_<float>(uv_mat.size());

  // Raw data from the Kinect has int
  float* z_mat_iter = reinterpret_cast<float*>(z_mat.data);

  for (cv::Mat_<cv::Vec2f>::const_iterator uv_iter = uv_mat.begin<cv::Vec2f>(), uv_end = uv_mat.end<cv::Vec2f>();
      uv_iter != uv_end; ++uv_iter, ++z_mat_iter)
  {
    T depth_i = depth.at < T > ((int)(*uv_iter)[1], (int)(*uv_iter)[0]);

    if (cvIsNaN((float)depth_i) || (depth_i == std::numeric_limits < T > ::min())
        || (depth_i == std::numeric_limits < T > ::max()))
      *z_mat_iter = std::numeric_limits<float>::quiet_NaN();
    else
      *z_mat_iter = depth_i * scale;
  }
}

}
}


#endif /* DEPTH_TO_3D_HPP_ */
