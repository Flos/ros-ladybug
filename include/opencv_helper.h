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
#include "ladybug/image_colorsep.h"

void rotate(cv::Mat& src, double angle, cv::Mat& dst);
const sensor_msgs::ImagePtr createImgPtr(const ladybug::image_colorsep* message);

#endif /* OPENCV_HELPER_H_ */
