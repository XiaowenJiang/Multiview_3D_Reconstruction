#include<iostream>
#include<opencv2\calib3d\calib3d.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<features2d\features2d.hpp>
#include<vector>
#include"Utils.h"

#ifndef __PROJECTPOINTS_H__
#define __PROJECTPOINTS_H__
class ProjectPoints
{
private:
	cv::Mat inputimage;
	std::vector<cv::Point2f> originpoints;
	std::vector<cv::Point2f> projectpoints;
	std::vector<int> visibility;
public:
	ProjectPoints();
	~ProjectPoints();
	ProjectPoints(cv::Mat image, std::vector<cv::Point2f>originalpoints, std::vector<cv::Point3f> points3d, cv::Mat rotation, cv::Mat translation,
		cv::Mat intrinsic, cv::Mat distor_coeff);
	void ShowReprojectionErrors();
	//used for sba comparison
	void ShowDifferenceSBA(std::vector<cv::Point2f> pointsbeforesba);
	std::vector<cv::Point2f> getProjectpoints();
	std::vector<int> getVisibility();
	cv::Mat getProjectionImage();
};

#endif