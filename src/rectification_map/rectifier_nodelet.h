/*
 * image_publisher.h
 *
 *  Created on: 15.02.2014
 *      Author: fnolden
 */

#ifndef image_rectifier_nodelet_H_
#define image_rectifier_nodelet_H_
#include "ros/ros.h"
#include "ladybug/image.h"
#include "image_transport/image_transport.h"
#include "camera_info_manager/camera_info_manager.h"
#include "opencv_helper.h"
#include <boost/thread.hpp>
#include "topic_names.h"
#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"


namespace ladybug {

class Rectifier_nodelet : public nodelet::Nodelet {
public:
	Rectifier_nodelet();
	virtual void onInit();
	virtual ~Rectifier_nodelet();
	void callback(const sensor_msgs::ImageConstPtr &input);
	void callback_camera_info(const sensor_msgs::CameraInfo &input_cam_info);
	void calc_parameters(const sensor_msgs::CameraInfo &cam_info);
	void find_point(int max, cv::Mat& rectified_point, cv::Point2d& new_c);
	void zoom_image(cv::Point2d begin_rect, cv::Size& img_size, const cv::Mat &in, cv::Mat &out);

private:
	boost::mutex init_lock;
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	image_transport::ImageTransport *it_;
	image_transport::Publisher pub_;
	std::string node_name_;
	std::string subscribe_topic_;
	std::string publish_topic_;
	std::string filepath_;
	cv::Mat map_x;
	cv::Mat map_y;

	double zoom_factor_;
	bool rotate_;

	std::string frame_id_;

	ros::Publisher pub_info_;
	ros::Subscriber sub_info_;
	std::string subscriber_info_topic_;
	std::string publish_info_topic_;
	sensor_msgs::CameraInfo cam_info_msg;
	bool calculated_rectification_parameters;
	double cx,cy;

	void load_maps();
};

} /* namespace ladybug */
#endif /* image_rectified_publisher_H_ */
