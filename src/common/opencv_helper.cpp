#include "opencv_helper.h"

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
	out_msg.header = message->header;

	if(!message->raw.empty())
	{
		out_msg.image = cv::imdecode(message->raw, CV_LOAD_IMAGE_COLOR).adjustROI(message->border_top, message->border_bottem, message->border_left, message->border_right);
		out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	}else{
		cv::Mat r,g,b;
		r = cv::imdecode(message->r, CV_LOAD_IMAGE_GRAYSCALE).adjustROI(message->border_top, message->border_bottem, message->border_left, message->border_right);
		g = cv::imdecode(message->g, CV_LOAD_IMAGE_GRAYSCALE).adjustROI(message->border_top, message->border_bottem, message->border_left, message->border_right);
		b = cv::imdecode(message->b, CV_LOAD_IMAGE_GRAYSCALE).adjustROI(message->border_top, message->border_bottem, message->border_left, message->border_right);

		std::vector<cv::Mat> channels;

		channels.push_back(r);
		channels.push_back(g);
		channels.push_back(b);

		cv::merge(channels, image);
		r.release();
		g.release();
		b.release();
		channels.clear();
		out_msg.encoding = sensor_msgs::image_encodings::RGB8;
		out_msg.image = image; //image;
		image.release();
	}
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


