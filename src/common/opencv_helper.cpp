#include "opencv_helper.h"
#include <boost/algorithm/string/predicate.hpp>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>

#define SCALE12_16  16

/**
 * Rotate an image
 */
void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}

const sensor_msgs::ImagePtr createImgPtr(const ladybug::image *message){
	//get image and convert it
	cv::Mat image;
	cv_bridge::CvImage out_msg;

	cv::Rect border;

	out_msg.header = message->header;
	out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	//ROS_INFO("color encoding %s bayer %s", message->color_encoding.c_str(), message->bayer_encoding.c_str());

	if(boost::icontains(message->color_encoding, "jpg")){

		if(message->raw.empty()){
			//Color seperated
			cv::Mat r,g,b;

			if( message->color_encoding == "jpg" || message->color_encoding == "jpg8"){
				r = cv::imdecode(message->r, CV_LOAD_IMAGE_UNCHANGED);
				g = cv::imdecode(message->g, CV_LOAD_IMAGE_UNCHANGED);
				b = cv::imdecode(message->b, CV_LOAD_IMAGE_UNCHANGED);
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
			}
			else
//				if(message->color_encoding == "jpg12")
			{
				std::string error_message = ""+ message->color_encoding+" currently not supported";
				throw new std::runtime_error(error_message);
//
//				r = cv::imdecode(message->r, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH) * SCALE12_16;
//				g = cv::imdecode(message->g, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH) * SCALE12_16;
//				b = cv::imdecode(message->b, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH) * SCALE12_16;
//				out_msg.encoding = sensor_msgs::image_encodings::RGB16;
			}
//			else if(message->color_encoding == "jpg16"){
//				r = cv::imdecode(message->r, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
//				g = cv::imdecode(message->g, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
//				b = cv::imdecode(message->b, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
//				out_msg.encoding = sensor_msgs::image_encodings::RGB16;
//			}


			if(r.cols == 0 ){
				std::string error_message = ""+ message->color_encoding+" not supported from opencv";
				throw new std::runtime_error(error_message);
			}
			else{
				//3 channels merge image
				std::vector<cv::Mat> channels;

				channels.push_back(r);
				channels.push_back(g);
				channels.push_back(b);

				cv::merge(channels, image);
				r.release();
				g.release();
				b.release();

				channels.clear();
			}
		}
		else{

//			ROS_INFO("sinlge color jpg");
			image = cv::imdecode(message->raw, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
			out_msg.encoding = message->bayer_encoding;
		}

	}
	else{ //RAW
		if(message->raw.empty()){
			//Color seperated
			std::string error_message = "color separated raw images not supported";
			throw new std::runtime_error(error_message);
		}
		else{ //Preprocessed
			/*Check if encoding matches the expected OpenCV encoding */
			if( message->bayer_encoding == "BGR8"){
				image = cv::Mat(message->raw).reshape(3, message->height);
				out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			}
			else if( message->bayer_encoding == "BGR16"){
				image = cv::Mat(message->height, message->width, CV_16UC3);
				memcpy(image.data, message->raw.data(),message->raw.size());
				out_msg.encoding = sensor_msgs::image_encodings::BGR16;
			}
			else if(message->bayer_encoding == "BGRA8"){
				image = cv::Mat(message->raw).reshape(4, message->height);
				out_msg.encoding = sensor_msgs::image_encodings::BGRA8;
			}
			else if(message->bayer_encoding == "BGRA16" ){
				image = cv::Mat(message->height, message->width, CV_16UC4);
				memcpy(image.data, message->raw.data(),message->raw.size());
				out_msg.encoding = sensor_msgs::image_encodings::BGRA16;
			}


			// Bayer encodings
			else if(message->bayer_encoding == "BGGR8" ){
				image = cv::Mat(message->raw).reshape(4, message->height/2);
				cv::cvtColor(image,image, CV_BayerRG2RGB);
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
			}
			else if(message->bayer_encoding == "BGGR16" ){
				image = cv::Mat(message->height, message->width, CV_16UC1);
				memcpy(image.data, message->raw.data(),message->raw.size());
				cv::cvtColor(image,image, CV_BayerRG2RGB);
				out_msg.encoding = sensor_msgs::image_encodings::RGB16;
			}
			else if(message->bayer_encoding == "RGGB8" ){
				image = cv::Mat(message->raw).reshape(4, message->height/2);
				cv::cvtColor(image,image, CV_BayerBG2RGB);
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
			}
			else if(message->bayer_encoding == "RGGB16" ){
				image = cv::Mat(message->height, message->width, CV_16UC1);
				memcpy(image.data, message->raw.data(),message->raw.size());
				cv::cvtColor(image,image, CV_BayerBG2RGB);
				out_msg.encoding = sensor_msgs::image_encodings::RGB16;
			}
			else if(message->bayer_encoding == "MONO8" ){
				image = cv::Mat(message->raw).reshape(1, message->height);
				out_msg.encoding = sensor_msgs::image_encodings::MONO8;
			}
			else if(message->bayer_encoding == "MONO16" ){
				image = cv::Mat(message->raw).reshape(1, message->height);
				out_msg.encoding = sensor_msgs::image_encodings::MONO16;
			}
			else{
				image = cv::imdecode(message->raw, CV_LOAD_IMAGE_UNCHANGED );
				out_msg.encoding = sensor_msgs::image_encodings::BGR8;
			}
		}

	}

	border = cv::Rect(message->border_left, message->border_top, image.cols - message->border_right - message->border_left, image.rows - message->border_bottem - message->border_top);
	out_msg.image = image(border);

	image.release();
	//rotate(image, -90, image);
	return out_msg.toImageMsg();
}

const sensor_msgs::ImagePtr rectifyImage(const cv_bridge::CvImagePtr &cv_ptr, cv::Mat &map_x, cv::Mat &map_y ){
	cv::remap(cv_ptr->image, cv_ptr->image, map_x, map_y,0,0);

	return cv_ptr->toImageMsg();
}

void loadMat(cv::Mat &mat, std::string filename){
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::READ);
	fs["mat"] >> mat;
	fs.release();
}

void saveMat(cv::Mat &mat, std::string filename){
	cv::FileStorage fs(filename.c_str(), cv::FileStorage::WRITE);
	fs << "mat" << mat;
	fs.release();
}


