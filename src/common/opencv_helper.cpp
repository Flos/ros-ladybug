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
		out_msg.image = cv::imdecode(message->raw, CV_LOAD_IMAGE_COLOR);
		out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	}else{
		cv::Mat r,g,b;
		r = cv::imdecode(message->r, CV_LOAD_IMAGE_GRAYSCALE);
		g = cv::imdecode(message->g, CV_LOAD_IMAGE_GRAYSCALE);
		b = cv::imdecode(message->b, CV_LOAD_IMAGE_GRAYSCALE);

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

