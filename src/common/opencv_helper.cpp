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

const sensor_msgs::ImagePtr createImgPtr(const ladybug::image_colorsep *message){
	//get image and convert it

	cv::Mat r,g,b;
	r = cv::imdecode(message->r, CV_LOAD_IMAGE_GRAYSCALE);
	g = cv::imdecode(message->g, CV_LOAD_IMAGE_GRAYSCALE);
	b = cv::imdecode(message->b, CV_LOAD_IMAGE_GRAYSCALE);

	cv::Mat image;
	std::vector<cv::Mat> channels;

	channels.push_back(r);
	channels.push_back(g);
	channels.push_back(b);

	cv::merge(channels, image);
	//rotate(image, -90, image);

	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	out_msg.image = image; //image;
	out_msg.header = message->header;

	r.release();
	g.release();
	b.release();
	image.release();

	channels.clear();

	return out_msg.toImageMsg();
}

