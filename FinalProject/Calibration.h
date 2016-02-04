#include<iostream>
#include<opencv2\calib3d\calib3d.hpp>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>
#include<features2d\features2d.hpp>
#include<opencv2\ml\ml.hpp>
#include<vector>
#include"Utils.h"

#ifndef ___CALIBRATION_H__
#define ___CALIBRATION_H__
class Calibration 
{
public:
	Calibration ();
	~Calibration ();
	void FindCorners();
	void InitCorners3D(std::vector<std::vector<cv::Point3f>>* Corners3D, cv::Size ChessBoardSize, int NImages, float SquareSize);
	void Calibrate();
	void Undistort();
	double computeReprojectionErrors();
private:
	//size of chessboard picture
	int image_width = 500;
	int image_height = 500;
	////number of inner corner along X axis of chessboard
	int chessboard_width = 9;
	//number of inner corner along Y axis of chessboard
	int chessboard_height = 6;
	//distance of per block in mm
	float square_width = 25;
	cv::Size ChessBoardSize = cv::Size(chessboard_width, chessboard_height);
	//corner points
	int corner_points = chessboard_width*chessboard_height;
	//number of images
	int img_nums = 20;
	//store all corner points from images
	std::vector<std::vector<cv::Point2f>> allcorners;
	//calibration pattern points
	std::vector<std::vector<cv::Point3f>> object_points;
	//intrinsic matrix
	cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	//distortion coefficients
	cv::Mat distortion_coeff = cv::Mat::zeros(8, 1, CV_32FC1);
	//rotation vectors
	cv::vector<cv::Mat> rotation_vectors;
	//tranformation vectors
	cv::vector<cv::Mat> translation_vectors;
	//reprojection errors
	std::vector<float> perViewErrors;
};

#endif