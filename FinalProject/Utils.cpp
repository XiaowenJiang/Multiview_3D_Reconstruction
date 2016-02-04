#include"Utils.h"

cv::Mat GetFromYML(std::string filename, std::string blockname)
{
	cv::Mat result;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs[blockname] >> result;
	fs.release();
	return result;
}

cv::Mat ReadImg(int i, std::string directory)
{
	std::string filename = "";
	std::string suffix = ".JPG";
	std::string img = "\\IMG_";
	filename = directory + img + std::to_string(i) + suffix;
	return cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
}

cv::Mat ReadImgRGB(int i, std::string directory)
{
	std::string filename = "";
	std::string suffix = ".JPG";
	std::string img = "\\IMG_";
	filename = directory + img + std::to_string(i) + suffix;
	return cv::imread(filename, CV_LOAD_IMAGE_COLOR);
}

void WriteImg(int i, std::string directory,std::string nameprefix, cv::Mat img)
{
	std::string filename = "";
	std::string suffix = ".JPG";
	std::string imgname = "\\IMG_"+nameprefix+"_";
	filename = directory + imgname + std::to_string(i) + suffix;
	cv::imwrite(filename, img);
}

void WriteYML(std::string filename, std::string blockname, cv::Mat matrix)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << blockname << matrix;
	fs.release();
}

std::vector<cv::Point2f> KeypointsToPoints(std::vector<cv::KeyPoint> keypoints)
{
	std::vector<cv::Point2f> points;
	for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin();
		it != keypoints.end(); it++)
	{
		points.push_back(it->pt);
	}
	return points;
}

//check if the matrix is a valid rotational matrix
bool CheckCoherentRotation(cv::Mat_<double>& R) {
	if (fabsf(determinant(R)) - 1.0 > 1e-07) {
		std::cerr << "det(R) != +-1.0, this is not a rotation matrix\n";
		return false;
	}
	return true;
}

void DrawCrossHair(cv::Mat img, cv::Point2f point)
{
	cv::Point2f upleft = cv::Point2f(point.x - 3, point.y-3);
	cv::Point2f upright = cv::Point2f(point.x + 3, point.y-3);
	cv::Point2f downleft = cv::Point2f(point.x-3, point.y + 3);
	cv::Point2f downright = cv::Point2f(point.x+3, point.y + 3);
	cv::line(img, upleft, downright, cv::Scalar(255, 255, 255));
	cv::line(img, upright, downleft, cv::Scalar(255, 255, 255));
}