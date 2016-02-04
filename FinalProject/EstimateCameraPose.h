#include"Utils.h"

#ifndef __ESTIMATECAMERAPOSE_H__
#define __ESTIMATECAMERAPOSE_H__
class EstimateCameraPose
{
private:
	std::vector<cv::Point2f> matchedpoints1, matchedpoints2;
	cv::Matx33d intrinsic;
	cv::Mat fundamental;
	cv::Mat_<double> E;
	cv::Mat Mask;
	//first camera matrix
	cv::Matx34d M1;
	//second camera matrix
	std::vector<cv::Matx34d> M2;
	//Rotation matrix 
	cv::Mat rotation;
	//Translation matrix
	cv::Mat translation;
	//triangulated 3D points
	std::vector<std::vector < cv::Point3f>> points3d;
	//final 3D points
	std::vector<cv::Point3f> truepoints3d;
public:
	EstimateCameraPose();
	~EstimateCameraPose();
	//calculate fundamental matrix and essentrial matrix from matched points and intrinsic matrix
	EstimateCameraPose(std::vector<cv::Point2f> points1, std::vector<cv::Point2f> points2, cv::Mat K);
	cv::Mat getFundamental();
	cv::Mat_<double> getEssential();
	void calcCameraPoseFromE();
	cv::Mat getM1();
	cv::Mat getM2();
	cv::Mat getRotation();
	cv::Mat getTranslation();
	void setRotation(cv::Mat R);
	void setTranslation(cv::Mat t);
	std::vector < cv::Point3f> get3Dpoints();
};

#endif