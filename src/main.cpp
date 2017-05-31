//============================================================================
// Name        : main.cpp
// Author      : Tzutalin
// Version     :
// Copyright   : Tzutalin
// Description : Demo Opencv-RgbdOdometry(http://docs.opencv.org/3.2.0/df/ddc/classcv_1_1rgbd_1_1Odometry.html)
//============================================================================

#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv_odometry/rgbd.hpp>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

using namespace cv;
using namespace std;

// This is for Kinect or Xtion
#define FOCUS_LENGTH 525.0
#define CX 319.5
#define CY 239.5

const float MIN_DEPTH = 0.8f;        // in meters
const float MAX_DEPTH = 4.0f;        // in meters
const float MAX_DEPTH_DIFF = 0.08f;  // in meters
const float MAX_POINTS_PART = 0.09f;

// This is for TUM dataset
const float PIXEL_TO_METER_SCALEFACTOR = 0.0002;

// Visualize trajectroy
const float WINDOW_SIZE = 800;
const float VISUALIZATION_SCALE_FACTOR = 60.0f;

using namespace cv;
using namespace std;

bool testRGBD(const std::vector<std::string>& inputRGBPaths,
              const std::vector<std::string>& inputDepthPaths) {
  namedWindow("RGBD Color", WINDOW_AUTOSIZE);
  namedWindow("Normalized RGBD Depth", WINDOW_AUTOSIZE);
  namedWindow("RGBD Trajectory", WINDOW_AUTOSIZE);
  Mat traj = Mat::zeros(WINDOW_SIZE, WINDOW_SIZE, CV_8UC3);

  std::shared_ptr<rgbd::Odometry> odom;

  char text[100];
  int fontFace = FONT_HERSHEY_PLAIN;
  double fontScale = 1;
  int thickness = 1;
  cv::Point textOrg(10, 50);

  Mat rotationMatrix, tranlslationMatrix;

  float vals[] = {FOCUS_LENGTH, 0., CX, 0., FOCUS_LENGTH, CY, 0., 0., 1.};

  const Mat cameraMatrix = Mat(3, 3, CV_32FC1, vals);

  bool isFirst = true;
  for (int i = 0; i != inputRGBPaths.size() - 1; i++) {
    std::string rgb0Path = inputRGBPaths[i];
    std::string depth0Path = inputDepthPaths[i];

    std::string rgb1Path = inputRGBPaths[i + 1];
    std::string depth1Path = inputDepthPaths[i + 1];

    // Read images start
    Mat colorImage0 = imread(rgb0Path);
    Mat depth0 = imread(depth0Path, IMREAD_UNCHANGED);
    Mat colorImage1 = imread(rgb1Path);
    Mat depth1 = imread(depth1Path, IMREAD_UNCHANGED);

    if (colorImage0.empty() || depth0.empty() || colorImage1.empty() ||
        depth1.empty()) {
      std::cerr << "Not correct RGB-D images";
      return -1;
    }

    Mat grayImage0, grayImage1, depthFlt0, depthFlt1 /*in meters*/;
    cvtColor(colorImage0, grayImage0, COLOR_BGR2GRAY);
    cvtColor(colorImage1, grayImage1, COLOR_BGR2GRAY);
    depth0.convertTo(depthFlt0, CV_32FC1, PIXEL_TO_METER_SCALEFACTOR);
    depth1.convertTo(depthFlt1, CV_32FC1, PIXEL_TO_METER_SCALEFACTOR);
    // Read images end

    Mat rigidTransform;

    if (!odom) {
      vector<int> iterCounts(4);
      iterCounts[0] = 7;
      iterCounts[1] = 7;
      iterCounts[2] = 7;
      iterCounts[3] = 10;

      vector<float> minGradMagnitudes(4);
      minGradMagnitudes[0] = 12;
      minGradMagnitudes[1] = 5;
      minGradMagnitudes[2] = 3;
      minGradMagnitudes[3] = 1;

      odom = std::make_shared<rgbd::RgbdOdometry>(
          cameraMatrix, MIN_DEPTH, MAX_DEPTH, MAX_DEPTH_DIFF, iterCounts,
          minGradMagnitudes, MAX_POINTS_PART,
          rgbd::Odometry::RIGID_BODY_MOTION);

      std::cerr << "Init tracker" << std::endl;
    }

    bool isSuccess = odom->compute(grayImage0, depthFlt0, Mat(), grayImage1,
                                   depthFlt1, Mat(), rigidTransform);

    Mat rotationMat = rigidTransform(cv::Rect(0, 0, 3, 3)).clone();
    Mat translateMat = rigidTransform(cv::Rect(3, 0, 1, 3)).clone();
    // If compute successfully, then update rotationMatrix and tranlslationMatrix
    if (isSuccess == true) {
      if (isFirst == true) {
        rotationMatrix = rotationMat.clone();
        tranlslationMatrix = translateMat.clone();
        isFirst = false;
        continue;
      }

	  // Update Rt
      tranlslationMatrix = tranlslationMatrix + (rotationMatrix * translateMat);
      rotationMatrix = rotationMat * rotationMatrix;
    }

    // Visualize traj
    if (isFirst == false) {
      int x =
          int(VISUALIZATION_SCALE_FACTOR * tranlslationMatrix.at<double>(0)) +
          WINDOW_SIZE / 2;
      int y =
          int(VISUALIZATION_SCALE_FACTOR * tranlslationMatrix.at<double>(2)) +
          WINDOW_SIZE / 2;

      circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
      rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0),
                CV_FILLED);
      if (isSuccess == true) {
        sprintf(text, "Coordinates: x = %04fm y = %04fm z = %04fm",
                tranlslationMatrix.at<double>(0),
                tranlslationMatrix.at<double>(1),
                tranlslationMatrix.at<double>(2));
      } else {
        sprintf(text, "Fail to compute odometry");
      }

      putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),
              thickness, 8);
    }
    imshow("RGBD Trajectory", traj);
    imshow("RGBD Color", grayImage1);

    cv::Mat normalizeDepth;
    depthFlt1.convertTo(normalizeDepth, CV_8UC1, 255.0 / MAX_DEPTH);
    imshow("Normalized RGBD Depth", normalizeDepth);

    const Mat distCoeff(1, 5, CV_32FC1, Scalar(0));
    cv::waitKey(1);
  }
  return true;
}

void readAssocTextfile(std::string filename,
                       std::vector<std::string>& inputRGBPaths,
                       std::vector<std::string>& inputDepthPaths) {
  std::string line;
  std::ifstream in_stream(filename.c_str());
  while (!in_stream.eof()) {
    std::getline(in_stream, line);
    stringstream ss(line);
    std::string buf;
    int count = 0;
    while (ss >> buf) {
      count++;
      if (count == 2) {
        inputRGBPaths.push_back(buf);
      } else if (count == 4) {
        inputDepthPaths.push_back(buf);
      }
    }
  }
  in_stream.close();
}

int main(int argc, char** argv) {
  std::cerr << "test opencv rgbd, start loading assoc.txt" << std::endl;
  std::vector<std::string> inputRGBPaths, inputDepthPaths;
  readAssocTextfile("assoc.txt", inputRGBPaths, inputDepthPaths);
  std::cerr << "test opencv rgbd, end loading assoc.txt" << std::endl;
  if (testRGBD(inputRGBPaths, inputDepthPaths)) {
    return 0;
  }
  return -1;
}
