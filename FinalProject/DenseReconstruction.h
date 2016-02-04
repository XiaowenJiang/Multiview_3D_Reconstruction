#include<iostream>
#include<opencv2\calib3d\calib3d.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<features2d\features2d.hpp>
#include<opencv2\ml\ml.hpp>
#include<vector>
#include"Utils.h"

#ifndef __DENSERECONSTRUCTION_H__
#define __DENSERECONSTRUCTION_H__
class DenseReconstuction{
private:
	//rectified rotation matrices
	cv::Mat R1;
	cv::Mat R2;
	//rectified projection matrices
	cv::Mat M1, M2;
	//disparity-to-depth mapping matrix
	cv::Mat Q;
	std::vector<cv::Mat> map1, map2;
	cv::Mat warped1, warped2;
	cv::Mat Disparity;

public:
	DenseReconstuction();
	~DenseReconstuction();
	DenseReconstuction(cv::Mat Fundamental,cv::Mat inputimg1,
		cv::Mat inputimage2,std::vector<cv::Point2f>points1,
		std::vector<cv::Point2f>points2);
	void ShowDisparityMap();
	cv::Mat getDisparityMap();
};

#endif