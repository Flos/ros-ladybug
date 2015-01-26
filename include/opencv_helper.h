/*
 * opencv_helper.h
 *
 *  Created on: 10.02.2014
 *      Author: fnolden
 */

#ifndef OPENCV_HELPER_H_
#define OPENCV_HELPER_H_

#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cvwimage.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include "protobuf/imageMessage.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "ladybug/image.h"

void rotate90(cv::Mat& src, cv::Mat& dst);
const sensor_msgs::ImagePtr createImgPtr(const ladybug::image* message);
const sensor_msgs::ImagePtr rectifyImage(const cv_bridge::CvImagePtr &cv_ptr, cv::Mat &map_x, cv::Mat &map_y, bool rotate_up = false );
void loadMat(cv::Mat &mat, std::string filename);
void saveMat(cv::Mat &mat, std::string filename);

#endif /* OPENCV_HELPER_H_ */
